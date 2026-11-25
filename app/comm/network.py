"""Drone-to-drone message layer: predictive send, trust scoring, fault injection.

Node ids: ground 0, drone ``uav.uav_no`` (or ``index + 1``). The outside replay
attacker transmits as radio id -1 and is never a receiver.

Defaults (tuned against tests/test_comm_network.py, dt 0.05 s, cthr 200 m):
    err_tx 5.0 m, heartbeat 1 s, link_timeout 3 s, replay window 1000 ms,
    rssi_beta 0.1 (per-link EMA of RSSI z fed to trust),
    TrustTable alpha 0.05, init 0.8, trust_min 0.7, pred_err_ref 5.0, rssi_z_max 0.75.
Per-packet z with rssi_z_max 1.5-2.0 missed the 30 s spoof bound on most seeds.
With the smoothed z and 0.75, scripts/comm_benchmark.py (120 s, seed 100) detects a
2 m/s drag in 16.5 s (fixed_rate) / 35 s (predictive) on the tight cluster. On the
server layout fixed_rate detects it in 33 s; predictive does not within 115 s.
No false quarantine in any benchmark run.
"""
from collections import deque

import numpy as np

from app.comm.packet import FLAG_HEARTBEAT, PACKET_SIZE, Packet, decode, encode
from app.comm.predictor import K_HISTORY, ConstantVelocityPredictor
from app.comm.radio import RadioModel
from app.comm.trust import TrustTable

MODES = ("fixed_rate", "predictive")
GROUND_ID = 0
ATTACKER_ID = -1


def _seq_newer(seq: int, last: int) -> bool:
    return 0 < ((seq - last) & 0xFFFF) < 0x8000


class CommNetwork:
    def __init__(
        self,
        mode: str,
        cthr: float,
        dt: float,
        key: bytes,
        predictor=None,
        err_tx: float = 5.0,
        heartbeat_s: float = 1.0,
        link_timeout_s: float = 3.0,
        replay_window_ms: int = 1000,
        rssi_beta: float = 0.1,
        seed: int = 0,
        radio: RadioModel | None = None,
        trust: TrustTable | None = None,
    ) -> None:
        if mode not in MODES:
            raise ValueError(f"unknown comm mode: {mode}")
        if heartbeat_s >= link_timeout_s:
            raise ValueError("heartbeat_s must be < link_timeout_s")
        self.mode, self.cthr, self.dt, self.key = mode, cthr, dt, key
        self.predictor = predictor or ConstantVelocityPredictor()
        self.err_tx, self.heartbeat_s = err_tx, heartbeat_s
        self.link_timeout_s, self.replay_window_ms = link_timeout_s, replay_window_ms
        self.rssi_beta = rssi_beta
        self.radio = radio or RadioModel(cthr, seed=seed)
        self.trust = trust or TrustTable()
        self.rng = np.random.default_rng(seed)
        self.t = 0.0
        self.seq: dict[int, int] = {}
        self.sent: dict[int, deque] = {}          # node -> own sent (t, pos, vel)
        self.last_sent: dict[int, float] = {}
        self._prev_pos: dict[int, np.ndarray] = {}
        self.history: dict[tuple[int, int], deque] = {}  # (rx, peer) -> received (t, pos, vel)
        self.last_seq: dict[tuple[int, int], int] = {}
        self.last_heard: dict[tuple[int, int], float] = {}
        self.rssi_z: dict[tuple[int, int], float] = {}   # (rx, peer) -> EMA of RSSI z
        self.spoofs: dict[int, tuple[float, np.ndarray, float, float]] = {}
        self.detected: dict[int, float] = {}
        self.replay_pos: np.ndarray | None = None
        self.replay_delay_s = 0.0
        self.replay_queue: deque = deque()  # (send_at_t, raw)
        self.packets_sent = self.auth_failures = self.replays_rejected = 0
        self._belief_sum, self._belief_n = 0.0, 0
        self._honest_s, self._false_q_s = 0.0, 0.0

    @property
    def now_ms(self) -> int:
        return int(self.t * 1000)

    @staticmethod
    def node_id(uav, index: int) -> int:
        return uav.uav_no if uav.uav_no is not None else index + 1

    def inject_gps_spoof(self, drone_id: int, rate_mps: float = 2.0, max_offset_m: float = 100.0) -> None:
        a = self.rng.uniform(0.0, 2 * np.pi)
        self.spoofs[drone_id] = (self.t, np.array([np.cos(a), np.sin(a), 0.0]), rate_mps, max_offset_m)

    def inject_replay(self, pos: tuple[float, float, float], delay_s: float = 2.0) -> None:
        self.replay_pos = np.asarray(pos, dtype=float)
        self.replay_delay_s = delay_s

    def _nodes(self, uavs, ground) -> list[tuple[int, np.ndarray, np.ndarray, str]]:
        # Broadcast velocity is measured motion (what a GPS reports), not the physics state,
        # which can carry avoidance impulses on a drone that is holding position.
        nodes = []
        for i, u in enumerate(uavs):
            nid, pos = self.node_id(u, i), u.pos._data.copy()
            prev = self._prev_pos.get(nid)
            vel = (pos - prev) / self.dt if prev is not None else u.velocity._data.copy()
            self._prev_pos[nid] = pos
            nodes.append((nid, pos, vel, u.state))
        if ground is not None:
            nodes.append((GROUND_ID, ground.pos._data.copy(), np.zeros(3), "Free"))
        return nodes

    def _reported(self, nid: int, pos: np.ndarray) -> np.ndarray:
        if nid not in self.spoofs:
            return pos
        t0, direction, rate, cap = self.spoofs[nid]
        return pos + direction * min(rate * (self.t - t0), cap)

    def _send(self, nid, pos, vel, state) -> bytes | None:
        hist = self.sent.get(nid)
        flags = 0
        if self.mode == "predictive" and hist:
            if self.t - self.last_sent[nid] >= self.heartbeat_s - 1e-9:
                flags = FLAG_HEARTBEAT
            elif np.linalg.norm(self.predictor.predict(hist, self.t)[0] - pos) <= self.err_tx:
                return None
        seq = self.seq.get(nid, 0)
        self.seq[nid] = (seq + 1) & 0xFFFF
        pkt = Packet(nid, seq, self.now_ms, tuple(pos), tuple(vel), state, flags)
        raw = encode(pkt, self.key)
        # Self-prediction uses the decoded (quantized) values receivers will see.
        q = decode(raw, self.key)
        self.sent.setdefault(nid, deque(maxlen=K_HISTORY)).append(
            (self.t, np.array(q.pos), np.array(q.vel)))
        self.last_sent[nid] = self.t
        self.packets_sent += 1
        return raw

    def _receive(self, rx: int, rx_pos: np.ndarray, raw: bytes, rssi: float) -> None:
        pkt = decode(raw, self.key)
        if pkt is None:
            self.auth_failures += 1
            return
        peer, link = pkt.sender_id, (rx, pkt.sender_id)
        if peer == rx:  # own packet replayed back
            self.replays_rejected += 1
            return
        last = self.last_seq.get(link)
        fresh_seq = last is None or _seq_newer(pkt.seq, last)
        if not fresh_seq or abs(pkt.t_ms - self.now_ms) > self.replay_window_ms:
            # Count only: an outside attacker can replay anyone's packet, so never blame the sender.
            self.replays_rejected += 1
            return
        self.last_seq[link] = pkt.seq
        t_pkt, pos, vel = pkt.t_ms / 1000.0, np.array(pkt.pos), np.array(pkt.vel)
        hist = self.history.setdefault(link, deque(maxlen=K_HISTORY))
        residual = float(np.linalg.norm(self.predictor.predict(hist, t_pkt)[0] - pos)) if hist else 0.0
        d = float(np.linalg.norm(pos - rx_pos))
        z = (rssi - self.radio.expected_rssi(d)) / self.radio.rssi_sigma_db
        # Single-packet RSSI is too noisy to tell a slow drag from fading; average per link.
        zs = self.rssi_z.get(link, 0.0)
        self.rssi_z[link] = zs = zs + self.rssi_beta * (z - zs)
        self.trust.observe(rx, peer, residual, zs)
        hist.append((t_pkt, pos, vel))
        self.last_heard[link] = self.t

    def tick(self, uavs, ground) -> None:
        self.t += self.dt
        nodes = self._nodes(uavs, ground)
        senders = []
        for nid, pos, vel, state in nodes:
            raw = self._send(nid, self._reported(nid, pos), vel, state)
            if raw is not None:
                senders.append((nid, pos, raw))
        if self.replay_pos is not None:
            for nid, pos, raw in senders:
                # TODO: attacker records lossless within cthr; no RNG draw for its own reception.
                if np.linalg.norm(pos - self.replay_pos) < self.cthr:
                    self.replay_queue.append((self.t + self.replay_delay_s, raw))
            while self.replay_queue and self.replay_queue[0][0] <= self.t + 1e-9:
                senders.append((ATTACKER_ID, self.replay_pos, self.replay_queue.popleft()[1]))
        rx_pos = {nid: pos for nid, pos, _, _ in nodes}
        heard = self.radio.deliver(senders, [(nid, pos) for nid, pos, _, _ in nodes], self.dt)
        for rx, pkts in heard.items():
            for raw, rssi in pkts:
                self._receive(rx, rx_pos[rx], raw, rssi)
        self._update_metrics(nodes)

    def _linked(self, a: int, b: int) -> bool:
        return all(
            self.t - self.last_heard.get((x, y), -np.inf) <= self.link_timeout_s
            and self.trust.get(x, y) >= self.trust.trust_min
            for x, y in ((a, b), (b, a))
        )

    def _hearers(self) -> dict[int, set[int]]:
        out: dict[int, set[int]] = {}
        for (rx, peer), t in self.last_heard.items():
            if self.t - t <= self.link_timeout_s:
                out.setdefault(peer, set()).add(rx)
        return out

    def quarantined_ids(self) -> set[int]:
        return self.trust.quarantined(self._hearers())

    def suspect_ids(self) -> set[int]:
        return self.trust.suspect(self._hearers())

    def mean_trust(self, node_id: int) -> float:
        return self.trust.mean_trust(node_id)

    def components(self, uavs, ground) -> list[list[int]]:
        ids = [self.node_id(u, i) for i, u in enumerate(uavs)] + ([GROUND_ID] if ground is not None else [])
        seen, comps = set(), []
        for start in range(len(ids)):
            if start in seen:
                continue
            comp, stack = [], [start]
            seen.add(start)
            while stack:
                i = stack.pop()
                comp.append(i)
                for j in range(len(ids)):
                    if j not in seen and self._linked(ids[i], ids[j]):
                        seen.add(j)
                        stack.append(j)
            comps.append(sorted(comp))
        return comps

    def _update_metrics(self, nodes) -> None:
        true_pos = {nid: pos for nid, pos, _, _ in nodes}
        for (rx, peer), hist in self.history.items():
            if peer in self.spoofs or rx not in true_pos or peer not in true_pos or not self._linked(rx, peer):
                continue
            self._belief_sum += float(np.linalg.norm(self.predictor.predict(hist, self.t)[0] - true_pos[peer]))
            self._belief_n += 1
        q = self.quarantined_ids()
        # TODO: time_to_detect is the first quarantine, even if the node later drops out of quarantine.
        for nid in self.spoofs:
            if nid in q and nid not in self.detected:
                self.detected[nid] = self.t - self.spoofs[nid][0]
        honest = [nid for nid, *_ in nodes if nid != GROUND_ID and nid not in self.spoofs]
        self._honest_s += len(honest) * self.dt
        self._false_q_s += sum(nid in q for nid in honest) * self.dt

    @property
    def metrics(self) -> dict:
        t = max(self.t, 1e-9)
        detect = None
        if self.spoofs and all(n in self.detected for n in self.spoofs):
            detect = max(self.detected[n] for n in self.spoofs)
        return {
            "comm_mode": self.mode,
            "packets_sent": self.packets_sent,
            "bytes_sent": self.packets_sent * PACKET_SIZE,
            "packets_per_s": self.packets_sent / t,
            "bytes_per_s": self.packets_sent * PACKET_SIZE / t,
            "belief_error_m": self._belief_sum / self._belief_n if self._belief_n else 0.0,
            "auth_failures": self.auth_failures,
            "replays_rejected": self.replays_rejected,
            "dropped_congestion": self.radio.dropped_congestion,
            "time_to_detect_s": detect,
            "false_quarantine_rate": self._false_q_s / self._honest_s if self._honest_s else 0.0,
        }

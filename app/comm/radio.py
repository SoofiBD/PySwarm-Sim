import math

import numpy as np


class RadioModel:
    """Lossy, range-limited broadcast radio with RSSI noise and per-receiver airtime cap."""

    def __init__(
        self,
        cthr: float,
        base_loss: float = 0.02,
        p0_dbm: float = -40.0,
        path_loss_exp: float = 2.5,
        rssi_sigma_db: float = 4.0,
        airtime_bytes_per_s: float = 20000.0,
        seed: int = 0,
    ):
        self.cthr = cthr
        self.base_loss = base_loss
        self.p0_dbm = p0_dbm
        self.path_loss_exp = path_loss_exp
        self.rssi_sigma_db = rssi_sigma_db
        self.airtime_bytes_per_s = airtime_bytes_per_s
        self.rng = np.random.default_rng(seed)
        self.dropped_congestion = 0

    def delivery_prob(self, d: float) -> float:
        if d >= self.cthr:
            return 0.0
        half = 0.5 * self.cthr
        if d <= half:
            return 1.0 - self.base_loss
        return (1.0 - self.base_loss) * (self.cthr - d) / half

    def expected_rssi(self, d: float) -> float:
        return self.p0_dbm - 10.0 * self.path_loss_exp * math.log10(max(d, 1.0))

    def sample_rssi(self, d: float) -> float:
        return self.expected_rssi(d) + float(self.rng.normal(0.0, self.rssi_sigma_db))

    def deliver(self, senders, receivers, dt: float) -> dict:
        """Return receiver id -> [(raw, rssi)]. Sender ids need not be receivers."""
        budget = self.airtime_bytes_per_s * dt
        out = {}
        for rid, rpos in receivers:
            heard = []
            for sid, spos, raw in senders:
                if sid == rid:
                    continue
                d = float(np.linalg.norm(np.asarray(spos) - np.asarray(rpos)))
                if self.rng.random() < self.delivery_prob(d):
                    heard.append((raw, self.sample_rssi(d)))
            kept, used = [], 0
            for i in self.rng.permutation(len(heard)):
                size = len(heard[i][0])
                if used + size <= budget:
                    kept.append(heard[i])
                    used += size
                else:
                    self.dropped_congestion += 1
            out[rid] = kept
        return out

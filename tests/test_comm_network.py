import numpy as np
import pytest

from app.comm.network import CommNetwork
from app.comm.packet import DEV_KEY
from app.ground import Ground
from app.myMath.vector import Vector
from app.uav import UAV

DT = 0.05
CTHR = 200.0


def _swarm(hover: bool = False, numbered: bool = True):
    rng = np.random.default_rng(0)
    uavs, vels = [], []
    for i in range(6):
        p = rng.uniform(-75.0, 75.0, 2)
        # Coherent swarm: shared drift plus small jitter keeps the 150 m cluster together.
        v = np.zeros(3) if hover else np.array([1.0 + rng.uniform(-0.5, 0.5), 0.5 + rng.uniform(-0.5, 0.5), 0.0])
        u = UAV(pos=Vector(p[0], p[1], 20.0), cthr=CTHR)
        u.uav_no = i + 1 if numbered else None
        u.velocity = Vector(*v)
        uavs.append(u)
        vels.append(v)
    return uavs, vels, Ground(pos=Vector(0, 0, 0))


def _run(net, uavs, vels, ground, seconds, at=None):
    for k in range(round(seconds / DT)):
        if at and k == round(at[0] / DT):
            at[1]()
        for u, v in zip(uavs, vels):
            u.pos = Vector(*(u.pos._data + v * DT))
        net.tick(uavs, ground)


def _net(mode, **kw):
    return CommNetwork(mode, CTHR, DT, DEV_KEY, seed=0, **kw)


def test_predictive_sends_less():
    sent = {}
    for mode in ("fixed_rate", "predictive"):
        uavs, vels, ground = _swarm()
        net = _net(mode)
        _run(net, uavs, vels, ground, 60)
        sent[mode] = net.metrics
    assert sent["predictive"]["packets_sent"] < 0.2 * sent["fixed_rate"]["packets_sent"]
    assert sent["predictive"]["belief_error_m"] < 6.0


def test_silent_hover_stays_linked():
    uavs, vels, ground = _swarm(hover=True)
    net = _net("predictive")
    _run(net, uavs, vels, ground, 10)
    assert len(net.components(uavs, ground)) == 1


def test_spoof_detected_by_rssi():
    uavs, vels, ground = _swarm()
    net = _net("fixed_rate")
    _run(net, uavs, vels, ground, 60, at=(5.0, lambda: net.inject_gps_spoof(3)))
    assert 3 in net.quarantined_ids()
    assert net.metrics["time_to_detect_s"] is not None
    assert net.metrics["time_to_detect_s"] < 30


@pytest.mark.parametrize("mode", ["fixed_rate", "predictive"])
def test_no_false_quarantine(mode):
    uavs, vels, ground = _swarm()
    net = _net(mode)
    _run(net, uavs, vels, ground, 60)
    assert net.metrics["false_quarantine_rate"] == 0.0


def test_replay_rejected():
    uavs, vels, ground = _swarm()
    net = _net("fixed_rate")
    centre = tuple(np.mean([u.pos._data for u in uavs], axis=0))
    net.inject_replay(centre)
    _run(net, uavs, vels, ground, 20)
    assert net.metrics["replays_rejected"] > 0
    assert net.quarantined_ids() == set()


def test_seq_wrap():
    uavs, vels, ground = _swarm()
    net = _net("fixed_rate")
    net.seq[3] = 65530
    _run(net, uavs, vels, ground, 10)
    assert net.seq[3] < 65530
    assert net.metrics["replays_rejected"] == 0


def test_uav_no_none():
    uavs, vels, ground = _swarm(numbered=False)
    net = _net("fixed_rate")
    _run(net, uavs, vels, ground, 1)
    assert [CommNetwork.node_id(u, i) for i, u in enumerate(uavs)] == [1, 2, 3, 4, 5, 6]
    assert set(net.seq) == {0, 1, 2, 3, 4, 5, 6}


def test_bad_mode():
    with pytest.raises(ValueError):
        _net("ideal")


def test_heartbeat_invariant():
    with pytest.raises(ValueError):
        _net("predictive", heartbeat_s=3, link_timeout_s=3)


def test_replay_out_of_sender_range_does_not_penalize():
    # Drones 1 and 3 are 300 m apart (> cthr); the attacker between them relays stale packets.
    uavs = [UAV(pos=Vector(x, 0.0, 20.0), cthr=CTHR) for x in (0.0, 150.0, 300.0)]
    for i, u in enumerate(uavs):
        u.uav_no = i + 1
    net = _net("fixed_rate")
    net.inject_replay((150.0, 10.0, 20.0))
    _run(net, uavs, [np.zeros(3)] * 3, None, 20)
    assert net.metrics["replays_rejected"] > 0
    for rx, peer in ((1, 3), (3, 1)):
        assert net.trust.get(rx, peer) >= net.trust.trust_min


def test_turning_drone_sends_before_heartbeat():
    # One drone on a tight circle (r 10 m, 20 m/s, 40 m/s^2): straight-line guess is 5 m off after ~0.5 s.
    u = UAV(pos=Vector(10.0, 0.0, 20.0), cthr=CTHR)
    u.uav_no = 1
    net = _net("predictive")
    w, sends = 2.0, []
    for k in range(1, round(0.9 / DT)):
        a = w * k * DT
        u.pos = Vector(10.0 * np.cos(a), 10.0 * np.sin(a), 20.0)
        u.velocity = Vector(-20.0 * np.sin(a), 20.0 * np.cos(a), 0.0)
        before = net.packets_sent
        net.tick([u], None)
        if net.packets_sent > before:
            sends.append(net.t)
    # first send at t=dt, then at least one error-triggered send before the 1 s heartbeat
    assert len(sends) >= 2
    assert sends[1] < net.heartbeat_s


def test_broadcast_velocity_is_measured_motion():
    # A drone holding position reports zero velocity even if its physics state carries an impulse,
    # as a real GPS would; the internal velocity field is not what the radio sends.
    uavs, _, ground = _swarm(hover=True)
    uavs[0].velocity = Vector(30.0, 0.0, 0.0)
    net = _net("fixed_rate")
    for _ in range(3):
        net.tick(uavs, ground)
    _, _, vel = net.sent[1][-1]
    assert np.allclose(vel, 0.0, atol=0.01)

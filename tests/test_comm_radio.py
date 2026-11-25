import numpy as np
import pytest

from app.comm.packet import PACKET_SIZE
from app.comm.radio import RadioModel


def test_delivery_prob_and_rssi():
    r = RadioModel(cthr=100.0)
    assert r.delivery_prob(0) == pytest.approx(0.98)
    assert r.delivery_prob(101) == 0
    assert r.delivery_prob(75) == pytest.approx(0.49)
    assert r.expected_rssi(10) == pytest.approx(-65.0)


def _scene():
    pos = [np.array([float(i), 0.0, 0.0]) for i in range(3)]
    senders = [(i, pos[i], bytes([i]) * PACKET_SIZE) for i in range(3)]
    receivers = [(i, pos[i]) for i in range(3)]
    return senders, receivers


def test_same_seed_same_output_and_no_self_delivery():
    s, r = _scene()
    a = RadioModel(cthr=100.0, seed=3).deliver(s, r, 0.05)
    b = RadioModel(cthr=100.0, seed=3).deliver(s, r, 0.05)
    assert a == b
    for rid, pkts in a.items():
        assert all(raw[0] != rid for raw, _ in pkts)


def test_outside_sender_reaches_receivers():
    r = RadioModel(cthr=100.0, base_loss=0.0)
    out = r.deliver([(-1, np.zeros(3), b"x" * PACKET_SIZE)], [(1, np.ones(3))], 0.05)
    assert len(out[1]) == 1


def test_congestion_cap():
    s, _ = _scene()
    r = [(10 + i, np.array([float(i), 1.0, 0.0])) for i in range(3)]
    m = RadioModel(cthr=100.0, base_loss=0.0, airtime_bytes_per_s=32 / 0.05)
    out = m.deliver(s, r, 0.05)
    assert all(len(out[i]) == 1 for i, _ in r)
    assert m.dropped_congestion == 6

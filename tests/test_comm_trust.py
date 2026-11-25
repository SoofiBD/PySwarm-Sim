from app.comm.trust import TrustTable


def _train(t, receiver, peer, n, z):
    for _ in range(n):
        t.observe(receiver, peer, 0.0, z)


def test_good_observations_raise_trust():
    t = TrustTable()
    _train(t, 1, 2, 100, 0.0)
    assert t.get(1, 2) > 0.95


def test_bad_rssi_lowers_trust():
    t = TrustTable()
    _train(t, 1, 2, 100, 5.0)
    assert t.get(1, 2) < 0.1


def test_big_residual_is_bad():
    t = TrustTable()
    for _ in range(100):
        t.observe(1, 2, 16.0, 0.0)
    assert t.get(1, 2) < 0.1


def test_unseen_is_init_and_penalize():
    t = TrustTable()
    assert t.get(1, 2) == 0.5
    t.penalize(1, 2)
    assert abs(t.get(1, 2) - 0.3) < 1e-9
    t.penalize(1, 2, 1.0)
    assert t.get(1, 2) == 0.0


def test_single_hearer_suspect_not_quarantined():
    t = TrustTable()
    _train(t, 1, 9, 100, 5.0)
    assert t.quarantined({9: {1}}) == set()
    assert t.suspect({9: {1}}) == {9}


def test_majority_quarantines():
    t = TrustTable()
    _train(t, 1, 9, 100, 5.0)
    _train(t, 2, 9, 100, 5.0)
    _train(t, 3, 9, 100, 0.0)
    h = {9: {1, 2, 3}}
    assert t.quarantined(h) == {9}
    assert t.suspect(h) == set()


def test_half_is_not_majority():
    t = TrustTable()
    _train(t, 1, 9, 100, 5.0)
    _train(t, 2, 9, 100, 0.0)
    h = {9: {1, 2}}
    assert t.quarantined(h) == set()
    assert t.suspect(h) == {9}


def test_mean_trust():
    t = TrustTable()
    assert t.mean_trust(9) == 1.0
    t.penalize(1, 9, 0.5)
    assert t.mean_trust(9) == 0.0

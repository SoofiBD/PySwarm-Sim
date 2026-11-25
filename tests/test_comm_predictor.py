from collections import deque

import numpy as np
import pytest

from app.comm.predictor import (
    ConstantVelocityPredictor,
    TinyMLPPredictor,
    features,
    make_predictor,
)


def _zero_mlp():
    return TinyMLPPredictor(
        np.zeros((22, 16)), np.zeros(16), np.zeros((16, 3)), np.zeros(3)
    )


def _rand_mlp(seed=1):
    r = np.random.default_rng(seed)
    return TinyMLPPredictor(
        r.normal(size=(22, 16)), r.normal(size=16), r.normal(size=(16, 3)), r.normal(size=3)
    )


def _hist(offset=0.0):
    h = deque(maxlen=4)
    for i in range(4):
        h.append((float(i), np.array([i * 1.0, 2 * i, 0.0]) + offset, np.array([1.0, 2.0, 0.1 * i])))
    return h


def test_cv_extrapolates():
    h = deque([(0.0, np.zeros(3), np.array([1.0, 2.0, 0.0]))], maxlen=4)
    pos, vel = ConstantVelocityPredictor().predict(h, 2.0)
    assert np.allclose(pos, [2, 4, 0]) and np.allclose(vel, [1, 2, 0])


def test_zero_mlp_equals_cv():
    h = _hist()
    p1, v1 = _zero_mlp().predict(h, 5.0)
    p2, v2 = ConstantVelocityPredictor().predict(h, 5.0)
    assert np.array_equal(p1, p2) and np.array_equal(v1, v2)


def test_mlp_single_entry_is_cv():
    h = deque([(0.0, np.zeros(3), np.ones(3))], maxlen=4)
    p1, _ = _rand_mlp().predict(h, 1.0)
    p2, _ = ConstantVelocityPredictor().predict(h, 1.0)
    assert np.array_equal(p1, p2)


def test_n_params_small():
    assert _rand_mlp().n_params == 22 * 16 + 16 + 16 * 3 + 3 <= 2000


def test_features_len_and_offset_invariance():
    a, b = features(_hist(), 5.0), features(_hist(offset=1234.5), 5.0)
    assert a.shape == (22,)
    assert np.allclose(a, b)


def test_save_load_roundtrip(tmp_path):
    m = _rand_mlp()
    m.save(tmp_path / "w.npz")
    m2 = TinyMLPPredictor.load(tmp_path / "w.npz")
    h = _hist()
    assert np.array_equal(m.predict(h, 6.0)[0], m2.predict(h, 6.0)[0])


def test_make_predictor(tmp_path, monkeypatch):
    assert isinstance(make_predictor("cv"), ConstantVelocityPredictor)
    monkeypatch.setattr("app.comm.predictor.DEFAULT_WEIGHTS", tmp_path / "none.npz")
    with pytest.raises(FileNotFoundError):
        make_predictor("mlp")
    with pytest.raises(ValueError):
        make_predictor("x")

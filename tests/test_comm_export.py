from collections import deque

import numpy as np

from app.comm.predictor import DEFAULT_WEIGHTS, TinyMLPPredictor
from scripts.export_predictor_c import export, quantize


def test_quantize_roundtrip_within_half_step():
    w = np.random.default_rng(0).normal(size=(22, 16))
    q, scale = quantize(w)
    assert q.dtype == np.int8 and np.abs(q).max() <= 127
    assert np.all(np.abs(q * scale - w) <= scale / 2 + 1e-12)


def test_quantize_all_zero():
    q, scale = quantize(np.zeros(3))
    assert np.all(q == 0) and scale > 0


def _random_history(r):
    h, t = deque(maxlen=4), 0.0
    pos, vel = r.uniform(0, 800, 3), r.uniform(-15, 15, 3)
    for _ in range(4):
        h.append((t, pos.copy(), vel.copy()))
        gap = r.uniform(0.05, 1.0)
        t += gap
        pos = pos + vel * gap
        vel = vel + r.normal(scale=2.0, size=3)
    return h, h[-1][0] + r.uniform(0.05, 1.0)


def test_dequantized_mlp_close_to_float():
    f = TinyMLPPredictor.load(DEFAULT_WEIGHTS)
    dq = TinyMLPPredictor(*(np.multiply(*quantize(w)) for w in (f.w1, f.b1, f.w2, f.b2)))
    r = np.random.default_rng(1)
    for _ in range(100):
        h, t = _random_history(r)
        assert np.linalg.norm(f.predict(h, t)[0] - dq.predict(h, t)[0]) < 0.5


def test_header_has_dims(tmp_path):
    out = tmp_path / "predictor_weights.h"
    export(DEFAULT_WEIGHTS, out)
    text = out.read_text()
    assert "#define PREDICTOR_N_HIDDEN 16" in text
    assert "#define PREDICTOR_N_IN 22" in text
    assert "int8_t predictor_w1[22 * 16]" in text
    assert "float predictor_w1_scale" in text


def test_scale_literals_are_valid_c_floats(tmp_path):
    # An all-zero tensor gets scale 1.0, which must not be printed as "1f".
    src = tmp_path / "w.npz"
    f = TinyMLPPredictor.load(DEFAULT_WEIGHTS)
    np.savez(src, w1=f.w1, b1=np.zeros_like(f.b1), w2=f.w2, b2=f.b2)
    out = tmp_path / "predictor_weights.h"
    export(src, out)
    scales = [ln for ln in out.read_text().splitlines() if "_scale = " in ln]
    assert len(scales) == 4
    assert all("." in ln.split("= ")[1] or "e" in ln.split("= ")[1] for ln in scales), scales
    assert "predictor_b1_scale = 1.0f;" in out.read_text()

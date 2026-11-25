from collections import deque
from pathlib import Path

import numpy as np

K_HISTORY = 4
N_FEATURES = 22
OUT_SCALE = 10.0  # metres
DEFAULT_WEIGHTS = Path(__file__).with_name("predictor_weights.npz")

# (t_s, pos, vel) of one peer's received packets, oldest first.
History = deque


class ConstantVelocityPredictor:
    @staticmethod
    def predict(history: History, t: float) -> tuple[np.ndarray, np.ndarray]:
        t_last, pos, vel = history[-1]
        return pos + vel * (t - t_last), vel.copy()


def features(history: History, t: float) -> np.ndarray:
    """22 floats from the peer's own packets, relative to its latest one."""
    t_last, pos, vel = history[-1]
    out = np.zeros(N_FEATURES)
    for i in range(min(3, len(history) - 1)):
        t_o, p_o, v_o = history[-2 - i]
        out[7 * i : 7 * i + 3] = (p_o - pos) / 100.0
        out[7 * i + 3 : 7 * i + 6] = (v_o - vel) / 10.0
        out[7 * i + 6] = t_last - t_o
    out[21] = t - t_last
    return out


class TinyMLPPredictor:
    """22 -> 16 (tanh) -> 3 position residual on top of constant velocity."""

    def __init__(self, w1, b1, w2, b2):
        self.w1, self.b1, self.w2, self.b2 = (np.asarray(a, dtype=float) for a in (w1, b1, w2, b2))

    @classmethod
    def load(cls, path: Path) -> "TinyMLPPredictor":
        with np.load(path) as z:
            return cls(z["w1"], z["b1"], z["w2"], z["b2"])

    def save(self, path: Path) -> None:
        np.savez(path, w1=self.w1, b1=self.b1, w2=self.w2, b2=self.b2)

    @property
    def n_params(self) -> int:
        return self.w1.size + self.b1.size + self.w2.size + self.b2.size

    def predict(self, history: History, t: float) -> tuple[np.ndarray, np.ndarray]:
        pos, vel = ConstantVelocityPredictor.predict(history, t)
        if len(history) < 2:
            return pos, vel
        h = np.tanh(features(history, t) @ self.w1 + self.b1)
        return pos + OUT_SCALE * (h @ self.w2 + self.b2), vel


def make_predictor(kind: str) -> ConstantVelocityPredictor | TinyMLPPredictor:
    if kind == "cv":
        return ConstantVelocityPredictor()
    if kind == "mlp":
        return TinyMLPPredictor.load(DEFAULT_WEIGHTS)  # raises FileNotFoundError if absent
    raise ValueError(f"unknown predictor kind: {kind}")

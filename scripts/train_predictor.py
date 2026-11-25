"""Train the tiny peer-position predictor on ideal-mode simulation trajectories.

Each run spawns 6 drones / 8 goals like the server (different seed per run),
records true per-drone trajectories, then cuts them into packet histories with
random gaps and horizons. The model learns the residual over constant velocity.
"""
import argparse
import random
import time
from collections import deque
from pathlib import Path

import numpy as np

from app.comm.predictor import DEFAULT_WEIGHTS, K_HISTORY, OUT_SCALE, TinyMLPPredictor, features
from app.goal import Goal
from app.ground import Ground
from app.myMath.vector import Vector
from app.simulation_step import SimulationStep
from app.uav import UAV

DT = 0.05
CTHR = 200.0
N_HIDDEN = 16


def make_layout(seed: int, n_drones: int = 6, n_goals: int = 8):
    """Same spawn boxes as server.main._create_simulation, own seed."""
    rnd = random.Random(seed)
    ground = Ground(pos=Vector(400.0, 400.0, 0.0), cthr=CTHR)
    uavs = []
    for i in range(n_drones):
        u = UAV(pos=Vector(rnd.uniform(300, 500), rnd.uniform(300, 500), 50.0), cthr=CTHR, ground=ground)
        u.setUAVNo(i + 1)
        uavs.append(u)
    goals = []
    for j in range(n_goals):
        g = Goal(pos=Vector(rnd.uniform(50, 750), rnd.uniform(50, 750), 0.0))
        g.setGoalNo(j + 1)
        goals.append(g)
    return uavs, goals, ground


def simulate(seed: int, seconds: float) -> tuple[np.ndarray, np.ndarray]:
    """True (pos, vel) per tick, shape (ticks, drones, 3) each."""
    uavs, goals, ground = make_layout(seed)
    step = SimulationStep(CTHR, DT)
    pos, vel = [], []
    for _ in range(round(seconds / DT)):
        step.tick(uavs, goals, ground)
        pos.append([u.pos._data.copy() for u in uavs])
        vel.append([u.velocity._data.copy() for u in uavs])
    return np.array(pos), np.array(vel)


def make_samples(pos, vel, n_per_drone, rng):
    """Packet histories (decimetre / cm/s quantized like the wire) -> (features, residual / OUT_SCALE, cv error)."""
    ticks, n_drones, _ = pos.shape
    qpos, qvel = np.round(pos * 10) / 10, np.round(vel * 100) / 100
    X, Y, cv_err = [], [], []
    for d in range(n_drones):
        for _ in range(n_per_drone):
            n_hist = int(rng.choice([2, 3, 4], p=[0.15, 0.15, 0.7]))
            gaps = np.maximum(1, np.round(rng.uniform(0.05, 1.0, n_hist - 1) / DT)).astype(int)
            ahead = max(1, round(rng.uniform(0.05, 1.0) / DT))
            last = int(rng.integers(gaps.sum(), ticks - ahead))
            idx = [last - int(s) for s in np.concatenate([[0], np.cumsum(gaps)])][::-1]
            hist = deque(((i * DT, qpos[i, d], qvel[i, d]) for i in idx), maxlen=K_HISTORY)
            t = (last + ahead) * DT
            cv = qpos[last, d] + qvel[last, d] * ahead * DT
            X.append(features(hist, t))
            Y.append((pos[last + ahead, d] - cv) / OUT_SCALE)
            cv_err.append(np.linalg.norm(pos[last + ahead, d] - cv))
    return np.array(X), np.array(Y), np.array(cv_err)


def train(X, Y, epochs, rng, lr=3e-3, batch=256):
    """Plain numpy Adam on MSE."""
    params = [rng.normal(scale=1 / np.sqrt(X.shape[1]), size=(X.shape[1], N_HIDDEN)), np.zeros(N_HIDDEN),
              rng.normal(scale=0.01, size=(N_HIDDEN, 3)), np.zeros(3)]
    m = [np.zeros_like(p) for p in params]
    v = [np.zeros_like(p) for p in params]
    step = 0
    for _ in range(epochs):
        order = rng.permutation(len(X))
        for s in range(0, len(X), batch):
            xb, yb = X[order[s:s + batch]], Y[order[s:s + batch]]
            w1, b1, w2, b2 = params
            h = np.tanh(xb @ w1 + b1)
            g_out = 2 * (h @ w2 + b2 - yb) / len(xb)
            g_h = (g_out @ w2.T) * (1 - h * h)
            grads = [xb.T @ g_h, g_h.sum(0), h.T @ g_out, g_out.sum(0)]
            step += 1
            for i, g in enumerate(grads):
                m[i] = 0.9 * m[i] + 0.1 * g
                v[i] = 0.999 * v[i] + 0.001 * g * g
                params[i] -= lr * (m[i] / (1 - 0.9 ** step)) / (np.sqrt(v[i] / (1 - 0.999 ** step)) + 1e-8)
    return TinyMLPPredictor(*params)


def mlp_error(model: TinyMLPPredictor, X, Y) -> np.ndarray:
    pred = np.tanh(X @ model.w1 + model.b1) @ model.w2 + model.b2
    return np.linalg.norm(pred - Y, axis=1) * OUT_SCALE


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--runs", type=int, default=20)
    ap.add_argument("--seed", type=int, default=0, help="first run seed; runs use seed..seed+runs-1")
    ap.add_argument("--seconds", type=float, default=120.0)
    ap.add_argument("--samples", type=int, default=600, help="samples per drone per run")
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--out", type=Path, default=DEFAULT_WEIGHTS)
    args = ap.parse_args()
    t0 = time.time()
    rng = np.random.default_rng(args.seed)
    n_train = max(1, round(0.8 * args.runs))  # split by run, not by sample
    sets = {"train": [], "test": []}
    for k in range(args.runs):
        pos, vel = simulate(args.seed + k, args.seconds)
        sets["train" if k < n_train else "test"].append(make_samples(pos, vel, args.samples, rng))
    Xtr, Ytr, _ = (np.concatenate(a) for a in zip(*sets["train"]))
    model = train(Xtr, Ytr, args.epochs, rng)
    model.save(args.out)
    print(f"runs {args.runs} (train {n_train}), {len(Xtr)} train samples, "
          f"{model.n_params} params, {time.time() - t0:.0f} s")
    if sets["test"]:
        Xte, Yte, cv_err = (np.concatenate(a) for a in zip(*sets["test"]))
        err = mlp_error(model, Xte, Yte)
        print(f"held-out mean position error: cv {cv_err.mean():.2f} m, mlp {err.mean():.2f} m "
              f"(median cv {np.median(cv_err):.2f} m, mlp {np.median(err):.2f} m; "
              f"rms cv {np.sqrt(np.mean(cv_err ** 2)):.2f} m, mlp {np.sqrt(np.mean(err ** 2)):.2f} m)")
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()

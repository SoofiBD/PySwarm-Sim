"""Compare comm modes and predictors on two layouts, with and without a GPS spoof.

tight:   the compact drifting cluster from tests/test_comm_network.py (no goals).
default: the server's spawn layout running the real SimulationStep with goals.

Each config runs twice: a clean run (traffic, belief error, false quarantine)
and a run where drone 3 is GPS-spoofed at 5 s (time to detect, false quarantine).
"""
import argparse

import numpy as np

from app.comm.network import CommNetwork
from app.comm.packet import DEV_KEY
from app.comm.predictor import make_predictor
from app.ground import Ground
from app.myMath.vector import Vector
from app.simulation_step import SimulationStep
from app.uav import UAV
from server.main import _create_simulation

DT = 0.05
CTHR = 200.0
SPOOF_AT_S, SPOOF_ID = 5.0, 3
CONFIGS = [("fixed_rate", "cv"), ("predictive", "cv"), ("predictive", "mlp")]


def tight(seed: int):
    """Same shape as tests/test_comm_network.py::_swarm, seeded separately."""
    rng = np.random.default_rng(seed)
    uavs, vels = [], []
    for i in range(6):
        p = rng.uniform(-75.0, 75.0, 2)
        v = np.array([1.0 + rng.uniform(-0.5, 0.5), 0.5 + rng.uniform(-0.5, 0.5), 0.0])
        u = UAV(pos=Vector(p[0], p[1], 20.0), cthr=CTHR)
        u.uav_no = i + 1
        u.velocity = Vector(*v)
        uavs.append(u)
        vels.append(v)
    ground = Ground(pos=Vector(0, 0, 0))

    def step(net):
        for u, v in zip(uavs, vels):
            u.pos = Vector(*(u.pos._data + v * DT))
        net.tick(uavs, ground)
    return step


def default(_seed: int):
    uavs, goals, ground = _create_simulation(cthr=CTHR)  # fixed server layout (random.seed(42))
    sim = None

    def step(net):
        nonlocal sim
        sim = sim or SimulationStep(CTHR, DT, comm=net)
        sim.tick(uavs, goals, ground)
    return step


def run(layout, mode, pred, seconds, seed, spoof):
    net = CommNetwork(mode, CTHR, DT, DEV_KEY, predictor=make_predictor(pred), seed=seed)
    step = layout(seed)
    for k in range(round(seconds / DT)):
        if spoof and k == round(SPOOF_AT_S / DT):
            net.inject_gps_spoof(SPOOF_ID)
        step(net)
    return net.metrics


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seconds", type=float, default=120.0)
    ap.add_argument("--seed", type=int, default=100, help="comm/radio seed (and tight layout seed)")
    args = ap.parse_args()
    detect_window = args.seconds - SPOOF_AT_S
    print("| layout | mode / predictor | packets/s | bytes/s | belief error (m) "
          "| spoof time to detect | false quarantine (clean / spoof) |")
    print("|---|---|---:|---:|---:|---|---|")
    for name, layout in (("tight", tight), ("default", default)):
        for mode, pred in CONFIGS:
            clean = run(layout, mode, pred, args.seconds, args.seed, spoof=False)
            spoof = run(layout, mode, pred, args.seconds, args.seed, spoof=True)
            ttd = spoof["time_to_detect_s"]
            ttd_s = f"{ttd:.1f} s" if ttd is not None else f"not detected in {detect_window:.0f} s"
            print(f"| {name} | {mode} / {pred} | {clean['packets_per_s']:.1f} | {clean['bytes_per_s']:.0f} "
                  f"| {clean['belief_error_m']:.2f} | {ttd_s} "
                  f"| {clean['false_quarantine_rate']:.3f} / {spoof['false_quarantine_rate']:.3f} |",
                  flush=True)


if __name__ == "__main__":
    main()

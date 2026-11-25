import numpy as np
import pytest

from app.comm.network import CommNetwork
from app.goal import Goal
from app.ground import Ground
from app.myMath.vector import Vector
from app.simulation_step import SimulationStep
from app.uav import UAV


def _world():
    rng = np.random.default_rng(0)
    uavs = []
    for i in range(5):
        p = rng.uniform(100, 200, 2)
        u = UAV(pos=Vector(p[0], p[1], 20.0), cthr=200.0)
        u.uav_no = i + 1
        uavs.append(u)
    goals = [Goal(pos=Vector(*rng.uniform(100, 300, 2), 20.0)) for _ in range(4)]
    return uavs, goals, Ground(pos=Vector(150, 150, 0), cthr=200.0)


def _run(step, ticks=200):
    uavs, goals, ground = _world()
    for _ in range(ticks):
        step.tick(uavs, goals, ground)
    return [u.pos._data.copy() for u in uavs]


def test_ideal_default_has_no_comm_and_never_touches_network(monkeypatch):
    def boom(*a, **k):
        raise AssertionError("comm used in ideal mode")

    monkeypatch.setattr(CommNetwork, "tick", boom)
    monkeypatch.setattr(CommNetwork, "components", boom)
    default, explicit = SimulationStep(), SimulationStep(comm_mode="ideal")
    assert default.comm is None and explicit.comm is None
    a, b = _run(default), _run(explicit)
    assert all(np.array_equal(x, y) for x, y in zip(a, b))
    assert "comm" not in default.metrics


def test_predictive_runs_and_reports_metrics():
    step = SimulationStep(comm_mode="predictive")
    assert step.comm is not None
    _run(step)
    assert step.metrics["comm"]["packets_sent"] > 0


def test_quarantined_uav_gets_no_target(monkeypatch):
    step = SimulationStep(comm_mode="predictive")
    monkeypatch.setattr(step.comm, "quarantined_ids", lambda: {3})
    uavs, goals, ground = _world()
    for _ in range(20):
        step.tick(uavs, goals, ground)
    assert uavs[2].getTarget() is None
    assert any(u.getTarget() is not None for u in uavs)


def test_no_relay_storm_before_links_exist():
    # Tick 1 has no links yet, so every node looks isolated; relays must wait for link_timeout.
    from server.main import _create_simulation
    uavs, goals, ground = _create_simulation()
    step = SimulationStep(comm_mode="predictive")
    for _ in range(round(step.comm.link_timeout_s / step.dt)):
        step.tick(uavs, goals, ground)
        assert not any(u.getState() == "Slave" for u in uavs)


def test_quarantined_uav_not_used_as_relay(monkeypatch):
    from app.manage_drones import DroneManager
    step = SimulationStep(comm_mode="predictive")
    step.comm.t = step.comm.link_timeout_s  # past warm-up
    seen = []
    monkeypatch.setattr(step.comm, "quarantined_ids", lambda: {3})
    monkeypatch.setattr(step.comm, "components", lambda uavs, ground: [[0, 1], [2], [3, 4, 5]])
    monkeypatch.setattr(DroneManager, "AssignRelay",
                        lambda *a, components=None, **k: seen.append(components) or [])
    uavs, goals, ground = _world()
    step.tick(uavs, goals, ground)
    assert seen == [[[0, 1], [3, 4, 5]]]  # index 2 is node 3


def test_hovering_drone_has_zero_velocity():
    # A drone that is not flown this tick (Free, no target) must not report stale velocity.
    uavs, _, ground = _world()
    uavs[0].velocity = Vector(30.0, 0.0, 0.0)
    step = SimulationStep()
    step.tick(uavs, [], ground)
    assert np.allclose(uavs[0].velocity._data, 0.0)


def test_predictive_on_server_layout_has_no_false_quarantine():
    from server.main import _create_simulation
    uavs, goals, ground = _create_simulation()
    step = SimulationStep(comm_mode="predictive")
    for k in range(round(60 / step.dt)):
        step.tick(uavs, goals, ground)
        if k < 20:
            assert not any(u.getState() == "Slave" for u in uavs)
    assert step.metrics["comm"]["false_quarantine_rate"] == 0.0

import asyncio
import pytest
import sys
sys.path.insert(0, '.')

from app.myMath.vector import Vector
from app.uav import UAV
from app.goal import Goal
from app.ground import Ground
from server.simulation_runner import SimulationRunner


@pytest.mark.asyncio
async def test_one_tick_does_not_raise():
    uavs = [UAV(pos=Vector(400, 400, 50))]
    goals = [Goal(pos=Vector(200, 200, 50))]
    ground = Ground(pos=Vector(400, 400, 0), cthr=200.0)

    received = []

    async def fake_broadcast(u, g):
        received.append((u, g))

    runner = SimulationRunner(uavs, goals, ground, broadcast_fn=fake_broadcast, dt=0.05)
    runner._step()
    await runner.broadcast_fn(runner.uavs, runner.goals)
    assert len(received) == 1


@pytest.mark.asyncio
async def test_metrics_increment_per_step():
    uavs = [UAV(pos=Vector(400, 400, 50))]
    goals = [Goal(pos=Vector(200, 200, 50))]
    ground = Ground(pos=Vector(400, 400, 0), cthr=200.0)

    async def noop(u, g): pass

    runner = SimulationRunner(uavs, goals, ground, broadcast_fn=noop)
    assert runner.metrics["steps"] == 0
    runner._step()
    assert runner.metrics["steps"] == 1
    runner._step()
    assert runner.metrics["steps"] == 2

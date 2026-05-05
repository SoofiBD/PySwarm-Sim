"""
Async simulation runner.

Decouples the simulation tick from PySide6 / QThread so it can be driven
by an asyncio event loop inside the FastAPI server.  One step of the physics
loop runs synchronously (CPU-bound), then control yields back to asyncio so
WebSocket I/O can proceed without blocking.
"""

import asyncio
from typing import Awaitable, Callable, List, Optional

from app.goal import Goal
from app.ground import Ground
from app.simulation_step import SimulationStep
from app.uav import UAV


class SimulationRunner:
    """
    Runs one simulation step per `dt` seconds, then calls `broadcast_fn`
    with the current UAV / Goal lists so the caller can serialise and push.
    """

    def __init__(
        self,
        uavs: List[UAV],
        goals: List[Goal],
        ground: Optional[Ground],
        broadcast_fn: Callable[[List[UAV], List[Goal]], Awaitable[None]],
        dt: float = 0.05,
        cthr: float = 200.0,
    ) -> None:
        self.uavs = uavs
        self.goals = goals
        self.ground = ground
        self.broadcast_fn = broadcast_fn
        self.dt = dt
        self.cthr = cthr
        self.running = False
        self._sim_step = SimulationStep(cthr=cthr, dt=dt)

    @property
    def metrics(self) -> dict:
        return self._sim_step.metrics

    def stop(self) -> None:
        self.running = False

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    async def run_loop(self) -> None:
        self.running = True
        while self.running:
            self._sim_step.tick(self.uavs, self.goals, self.ground)
            await self.broadcast_fn(self.uavs, self.goals)
            await asyncio.sleep(self.dt)

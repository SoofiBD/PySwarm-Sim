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
from app.logger import log_simulation_event
from app.manage_drones import CollisionDetector, DroneManager, DroneNetwork
from app.myMath.matrixOperation import MatrixOperation
from app.pathfinding import AStarPathfinder
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
        self._step_count = 0
        self._pathfinder = AStarPathfinder(
            grid_resolution=40.0,
            bounds=(0, 800, 0, 800, 0, 200),
            obstacle_radius=20.0,
        )

    def stop(self) -> None:
        self.running = False

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    async def run_loop(self) -> None:
        self.running = True
        while self.running:
            self._step()
            await self.broadcast_fn(self.uavs, self.goals)
            await asyncio.sleep(self.dt)

    # ------------------------------------------------------------------
    # Single simulation step  (pure Python / numpy — no asyncio inside)
    # ------------------------------------------------------------------
    def _step(self) -> None:
        if not self.uavs:
            return

        self._step_count += 1

        # 1. Build adjacency / distance matrix
        if self.ground:
            adj = MatrixOperation.UAVtoGround_AdjMatrix(self.uavs, self.ground)
        else:
            adj = MatrixOperation.UAVtoUAV_AdjMatrix(self.uavs)

        # 2. Reset transient state before re-assigning targets / roles
        DroneManager.ResetUAVs(self.uavs)
        DroneManager.ResetGoals(self.goals)

        # 3. Drone-to-drone target sharing over comm graph
        DroneNetwork.share_targets(self.uavs)

        # 4. Target assignment — each free drone picks its closest free goal
        i = 0
        free_uavs = DroneManager.GetFreeUAVNumber(self.uavs)
        free_goals = DroneManager.GetFreeGoalNumber(self.goals)
        iterations = 0
        while free_uavs > 0 and free_goals > 0 and iterations < len(self.uavs):
            uav = self.uavs[i % len(self.uavs)]
            if uav.getState() == "Free":
                target = DroneManager.AssignTarget(self.goals, self.uavs, uav)
                if target:
                    uav.setTarget(target)
                free_uavs -= 1
                free_goals -= 1
            i += 1
            iterations += 1

        # 5. Connectivity check & relay assignment
        components = MatrixOperation.find_conn_comp(adj, self.cthr)
        if self._step_count % 10 == 0:  # log every 10 steps to reduce noise
            log_simulation_event("CONNECTIVITY", f"{len(components)} component(s)")

        if len(components) != 1:
            DroneManager.AssignRelay(self.uavs, adj, self.ground, self.cthr)
            for uav in self.uavs:
                if uav.getState() == "Slave":
                    uav.followLeader(dt=self.dt)

        # 6. Collision avoidance (KD-tree accelerated — O(N log N))
        CollisionDetector.apply_avoidance(self.uavs)

        # 7. Physics-based movement toward assigned target
        for uav in self.uavs:
            target = uav.getTarget()
            if target and uav.getState() == "Leader":
                positions = [u.pos for u in self.uavs if u is not uav]
                uav.move(target, dt=self.dt,
                         pathfinder=self._pathfinder,
                         all_uav_positions=positions)

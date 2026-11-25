"""
Shared simulation step logic.

Single source of truth for the per-tick simulation update.  Both the
desktop QThread engine (app/engine.py) and the async FastAPI runner
(server/simulation_runner.py) delegate to SimulationStep.tick().
"""

from typing import List, Optional


from app.comm.network import GROUND_ID, CommNetwork
from app.comm.packet import load_swarm_key
from app.goal import Goal
from app.ground import Ground
from app.logger import log_simulation_event
from app.manage_drones import CollisionDetector, DroneManager
from app.myMath.matrixOperation import MatrixOperation
from app.myMath.vector import Vector
from app.pathfinding import AStarPathfinder
from app.uav import UAV


class SimulationStep:
    """
    Encapsulates one simulation tick.

    Parameters
    ----------
    cthr : float
        Communication threshold in metres.
    dt : float
        Timestep in seconds.
    comm_mode : str
        "ideal" (omniscient, default), "fixed_rate" or "predictive".
    comm : CommNetwork, optional
        Prebuilt network; built from comm_mode when not ideal and omitted.
    """

    def __init__(self, cthr: float = 200.0, dt: float = 0.05,
                 comm_mode: str = "ideal", comm: Optional[CommNetwork] = None) -> None:
        self.cthr = cthr
        self.dt = dt
        self.comm = comm
        if self.comm is None and comm_mode != "ideal":
            self.comm = CommNetwork(comm_mode, cthr, dt, load_swarm_key())
        self._step_count = 0
        self.metrics: dict = {
            "steps": 0,
            "collisions_resolved": 0,
            "targets_visited": 0,
        }
        self._pathfinder = AStarPathfinder(
            grid_resolution=40.0,
            bounds=(0, 800, 0, 800, 0, 200),
            obstacle_radius=20.0,
        )

    def tick(
        self,
        uavs: List[UAV],
        goals: List[Goal],
        ground: Optional[Ground],
    ) -> None:
        """Execute one full simulation step."""
        if not uavs:
            return

        self._step_count += 1
        self.metrics["steps"] = self._step_count
        quarantined = self.comm.quarantined_ids() if self.comm else set()

        # 1. Build adjacency / distance matrix
        if ground:
            adj = MatrixOperation.UAVtoGround_AdjMatrix(uavs, ground)
        else:
            adj = MatrixOperation.UAVtoUAV_AdjMatrix(uavs)

        # 2. Clean up completed missions (NOT a full reset)
        DroneManager.ResetUAVs(uavs)
        DroneManager.ResetGoals(goals, uavs)

        # 3. Age free goals so long-waiting targets outrank closer newcomers
        #    (composite time x distance valuation, see app/valuation.py)
        for goal in goals:
            if goal.state == "Free":
                goal.age_seconds += self.dt

        # 4. Target assignment — compute distance matrix once, assign globally by valuation
        #    DroneManager.AssignTarget already computes composite_value; we precompute adjacency once
        #    and iterate free UAVs in order, assigning highest-value goal per UAV. O(D·G) total not O(D²·G)
        if any(g.state == "Free" for g in goals):
            uav_to_goal = MatrixOperation.UAVtoGoal_AdjMatrix(uavs, goals) if goals and uavs else None
            free_uavs_list = [
                u for i, u in enumerate(uavs)
                if u.getState() == "Free"
                and (not quarantined or CommNetwork.node_id(u, i) not in quarantined)
            ]
            for uav in free_uavs_list:
                if DroneManager.GetFreeGoalNumber(goals) == 0:
                    break
                # reuse precomputed distances to avoid re-building matrix inside AssignTarget
                target = DroneManager.AssignTarget(goals, uavs, uav, dist_matrix=uav_to_goal)
                if target:
                    uav.setTarget(target)

        # 5. Connectivity check & relay assignment
        warming_up = False
        if self.comm:
            components = self.comm.components(uavs, ground)
            # Quarantined nodes are left out of the graph, so they never bridge or get bridged.
            ids = [CommNetwork.node_id(u, i) for i, u in enumerate(uavs)] + [GROUND_ID]
            components = [c for c in ([i for i in comp if ids[i] not in quarantined]
                                      for comp in components) if c]
            # No links exist until peers have had time to hear each other; every node would look cut off.
            warming_up = self.comm.t < self.comm.link_timeout_s
        else:
            components = MatrixOperation.find_conn_comp(adj, self.cthr)
        if self._step_count % 10 == 0:
            log_simulation_event("CONNECTIVITY", f"{len(components)} component(s)")

        if len(components) > 1 and not warming_up:
            DroneManager.AssignRelay(uavs, adj, ground, self.cthr,
                                     components=components if self.comm else None)
            for uav in uavs:
                if uav.getState() == "Slave":
                    uav.followLeader(dt=self.dt)

        # 6. Collision avoidance
        resolved = CollisionDetector.apply_avoidance(uavs)
        self.metrics["collisions_resolved"] += resolved

        # 7. Physics-based movement toward assigned target
        for uav in uavs:
            target = uav.getTarget()
            if target and uav.getState() == "Leader":
                positions = [u.pos for u in uavs if u is not uav]
                uav.move(target, dt=self.dt,
                         pathfinder=self._pathfinder,
                         all_uav_positions=positions)

        # Idle drones never integrate velocity, so clear what they kept from their last flight.
        # Holding slaves keep theirs: it carries the collision-avoidance impulse.
        for uav in uavs:
            if uav.getState() == "Free" and uav.getTarget() is None:
                uav.velocity = Vector(0.0, 0.0, 0.0)

        if self.comm:
            self.comm.tick(uavs, ground)
            self.metrics["comm"] = self.comm.metrics

        self.metrics["targets_visited"] = sum(
            1 for g in goals if g.state == "Visited"
        )

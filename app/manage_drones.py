from typing import List, Optional, Tuple, Dict

import numpy as np
from scipy.spatial import cKDTree  # O(N log N) spatial queries

from app.myMath.matrixOperation import MatrixOperation
from app.uav import UAV
from app.goal import Goal
from app.ground import Ground
from app.valuation import TargetValuation


class DroneNetwork:
    """Manages drone-to-drone communication and target sharing."""

    COMM_RANGE = 150.0

    @staticmethod
    def build_communication_graph(uavs: List[UAV], adj: "np.ndarray | None" = None) -> Dict[int, List[int]]:
        if adj is None:
            adj = MatrixOperation.UAVtoUAV_AdjMatrix(uavs)
        graph: Dict[int, List[int]] = {i: [] for i in range(len(uavs))}
        # vectorized row scan
        for i in range(len(uavs)):
            neighbors = np.where((adj[i] > 0) & (adj[i] <= DroneNetwork.COMM_RANGE))[0]
            for j in neighbors:
                if j > i:  # add symmetrically once
                    graph[i].append(int(j))
                    graph[j].append(i)
        return graph

    @staticmethod
    def share_targets(uavs: List[UAV], adj: "np.ndarray | None" = None) -> None:
        graph = DroneNetwork.build_communication_graph(uavs, adj=adj)

        for uav in uavs:
            uav.known_targets = []

        for i, uav in enumerate(uavs):
            target = uav.getTarget()
            if target:
                uav.distance_to_target = float(MatrixOperation.distance(uav.pos, target.pos))
                for neighbor_idx in graph.get(i, []):
                    neighbor = uavs[neighbor_idx]
                    if neighbor.getState() == "Free":
                        dist = float(MatrixOperation.distance(neighbor.pos, target.pos))
                        if target not in neighbor.known_targets:
                            neighbor.known_targets.append(target)
                        neighbor.setDistanceToTarget(dist)

    @staticmethod
    def broadcast_position(uavs: List[UAV]) -> Dict[int, List[Tuple[int, Tuple[float, float, float]]]]:
        """Return per-UAV neighbor positions for comm-aware planning."""
        adj = MatrixOperation.UAVtoUAV_AdjMatrix(uavs)
        graph = DroneNetwork.build_communication_graph(uavs, adj=adj)
        all_pos: Dict[int, Tuple[float, float, float]] = {
            i: (u.pos.x, u.pos.y, u.pos.z) for i, u in enumerate(uavs)
        }
        result: Dict[int, List[Tuple[int, Tuple[float, float, float]]]] = {}
        for i in range(len(uavs)):
            result[i] = [(n, all_pos[n]) for n in graph.get(i, [])]
        return result


class CollisionDetector:
    """
    Drone collision detection and avoidance.

    Detection uses scipy's cKDTree so pair-finding is O(N log N) rather
    than the previous O(N²) distance-matrix scan.  For N=100 drones this
    is ~10× faster; at N=1000 the difference is ~100×.

    Avoidance applies a symmetric repulsion impulse that pushes each
    drone away from its closest offending neighbour along the line joining
    their positions.
    """

    MIN_SAFE_DISTANCE = 5.0

    @staticmethod
    def detect_potential_collisions(
        uavs: List[UAV],
    ) -> List[Tuple[UAV, UAV, float]]:
        """
        Return all (uavA, uavB, distance) triples where distance < MIN_SAFE_DISTANCE.

        Complexity: O(N log N) average via cKDTree.query_pairs.
        """
        if len(uavs) < 2:
            return []

        positions = np.array([u.pos._data for u in uavs], dtype=np.float64)
        tree = cKDTree(positions)
        # query_pairs returns a set of (i, j) index pairs with i < j
        pairs = tree.query_pairs(CollisionDetector.MIN_SAFE_DISTANCE)

        result: List[Tuple[UAV, UAV, float]] = []
        for i, j in pairs:
            dist = float(np.linalg.norm(positions[i] - positions[j]))
            if dist > 0:  # skip perfectly co-located (avoid zero-div in resolve)
                result.append((uavs[i], uavs[j], dist))
        return result

    REPULSION_SPEED: float = 2.0  # m/s impulse magnitude

    @staticmethod
    def resolve_collision(uav: UAV, other_uav: UAV) -> None:
        """
        Add a repulsion velocity impulse to `uav` pointing away from `other_uav`.
        Modifies velocity only — the physics integrator applies the displacement.
        Co-located drones receive a nudge along +X.
        """
        delta = uav.pos._data - other_uav.pos._data
        norm = float(np.linalg.norm(delta))
        if norm < 1e-9:
            direction = np.array([1.0, 0.0, 0.0])
        else:
            direction = delta / norm

        from app.myMath.vector import Vector
        impulse = direction * CollisionDetector.REPULSION_SPEED
        uav.velocity = Vector(*(uav.velocity._data + impulse))

    @staticmethod
    def apply_avoidance(uavs: List[UAV]) -> int:
        """
        Apply avoidance to all colliding pairs.
        Priority: busy drones (Leader/Slave) yield to free drones so that
        free drones are repelled rather than disrupting active missions.
        Returns number of pairs resolved.
        """
        collisions = CollisionDetector.detect_potential_collisions(uavs)
        resolved = 0
        for uav1, uav2, _ in collisions:
            if uav1.getState() != "Free":
                CollisionDetector.resolve_collision(uav2, uav1)
            elif uav2.getState() != "Free":
                CollisionDetector.resolve_collision(uav1, uav2)
            else:
                # Both free — push both symmetrically
                CollisionDetector.resolve_collision(uav1, uav2)
                CollisionDetector.resolve_collision(uav2, uav1)
            resolved += 1
        return resolved


class DroneManager:
    """Manages high-level UAV behaviors and assignments."""

    @staticmethod
    def AssignTarget(goals: List[Goal], uavs: List[UAV], this_uav: UAV, dist_matrix: "np.ndarray | None" = None) -> Optional[Goal]:
        if not goals:
            return None
        if dist_matrix is None:
            dist_matrix = MatrixOperation.UAVtoGoal_AdjMatrix(uavs, goals)
        try:
            this_uav_idx = uavs.index(this_uav)
        except ValueError:
            return None
        possible_targets: list[tuple[Goal, float]] = []
        for i, goal in enumerate(goals):
            if goal.state == "Free":
                dist_to_goal = float(dist_matrix[this_uav_idx, i]) if dist_matrix.size else float("inf")
                value = TargetValuation.composite_value(goal.age_seconds, dist_to_goal)
                possible_targets.append((goal, value))

        if possible_targets:
            possible_targets.sort(key=lambda x: x[1], reverse=True)
            assigned_goal = possible_targets[0][0]
            assigned_goal.state = "Visiting"
            assigned_goal.age_seconds = 0.0
            this_uav.state = "Leader"
            return assigned_goal
        return None

    @staticmethod
    def ResetUAVs(uavs: List[UAV]):
        """Reset only drones whose mission is complete or whose target is gone."""
        for uav in uavs:
            if uav.state == "Leader":
                target = uav.getTarget()
                # Target was visited or somehow lost — release the drone
                if target is None or target.state == "Visited":
                    uav.clearMySlaveList()
                    uav.target = None
                    uav.state = "Free"
            elif uav.state == "Slave":
                leader = uav.getMyLeader()
                # Leader no longer exists or became Free — release slave
                if leader is None or leader.state == "Free":
                    uav.my_leader = None
                    uav.state = "Free"
    
    @staticmethod
    def ResetGoals(goals: List[Goal], uavs: List[UAV]):
        """Free goals that are 'Visiting' but no drone is actually pursuing them."""
        # Build set of goals currently targeted by a Leader drone
        actively_targeted = set()
        for uav in uavs:
            if uav.state == "Leader" and uav.target is not None:
                actively_targeted.add(id(uav.target))

        for goal in goals:
            if goal.state == "Visiting" and id(goal) not in actively_targeted:
                goal.state = "Free"
    
    @staticmethod
    def GetFreeGoalNumber(goals: List[Goal]) -> int:
        return sum(1 for g in goals if g.state == "Free")

    @staticmethod
    def GetFreeUAVNumber(uavs: List[UAV]) -> int:
        return sum(1 for u in uavs if u.state == "Free")

    @staticmethod
    def ChangeSlaveList(slave_uav: UAV, leader_uav: UAV):
        """Transfers slaves from one UAV to another (dedup, target cleared)."""
        slaves = list(slave_uav.my_slave_list)
        for x in slaves:
            x.target = None
            x.state = "Slave"
            x.my_leader = leader_uav
            if x not in leader_uav.my_slave_list:
                leader_uav.my_slave_list.append(x)
        slave_uav.my_slave_list.clear()

    @staticmethod
    def _attach_slave(slave: UAV, leader: UAV, relay_uavs: List[UAV]) -> None:
        if slave is leader or slave.my_leader is leader:
            return
        # detach from previous leader
        if slave.my_leader and slave in slave.my_leader.my_slave_list:
            slave.my_leader.my_slave_list.remove(slave)
        slave.state = "Slave"
        slave.my_leader = leader
        if slave not in leader.my_slave_list:
            leader.my_slave_list.append(slave)
        if slave not in relay_uavs:
            relay_uavs.append(slave)

    @staticmethod
    def AssignRelay(uavs: List[UAV], adj: List[List[float]], ground: Optional[Ground] = None, cthr: float = 200) -> List[UAV]:
        """Idempotent relay assignment — avoids duplicate slave appends and oscillation."""
        relay_uavs: List[UAV] = []
        components = MatrixOperation.find_conn_comp(adj, cthr)
        if len(components) <= 1:
            return relay_uavs
        if ground:
            risky_links = MatrixOperation.findRiskyLinks_ground(components, cthr, uavs, ground)
        else:
            risky_links = MatrixOperation.findRiskyLinks(components, cthr, uavs)

        seen_pairs: set[tuple[int, int]] = set()
        for link in risky_links:
            idx1, idx2 = link[0], link[1]
            n_uavs = len(uavs)
            if idx1 >= n_uavs or idx2 >= n_uavs:
                continue
            pair_key = (min(idx1, idx2), max(idx1, idx2))
            if pair_key in seen_pairs:
                continue
            seen_pairs.add(pair_key)
            uav1, uav2 = uavs[idx1], uavs[idx2]
            # Skip if already slaves to same leader
            if uav1.state == "Slave" and uav2.state == "Slave" and uav1.my_leader is uav2.my_leader:
                continue

            if uav1.state == "Free" and uav2.state == "Free":
                # attach both to nearest leader
                leader = None
                min_dist = float("inf")
                for u in uavs:
                    if u.state == "Leader":
                        d = MatrixOperation.distance(u.pos, uav1.pos)
                        if d < min_dist:
                            min_dist = d
                            leader = u
                if leader is not None:
                    DroneManager._attach_slave(uav1, leader, relay_uavs)
                    DroneManager._attach_slave(uav2, leader, relay_uavs)

            elif uav1.state == "Leader" and uav2.state == "Leader":
                # farther from its target becomes slave
                d1 = MatrixOperation.distance(uav1.pos, uav1.target.pos) if uav1.target else float("inf")
                d2 = MatrixOperation.distance(uav2.pos, uav2.target.pos) if uav2.target else float("inf")
                if d1 > d2:
                    DroneManager.ChangeSlaveList(uav1, uav2)
                    if uav1.target:
                        uav1.target.state = "Free"
                        uav1.target = None
                    uav1.state = "Slave"
                    uav1.my_leader = uav2
                    if uav1 not in uav2.my_slave_list:
                        uav2.my_slave_list.append(uav1)
                    if uav1 not in relay_uavs:
                        relay_uavs.append(uav1)
                else:
                    DroneManager.ChangeSlaveList(uav2, uav1)
                    if uav2.target:
                        uav2.target.state = "Free"
                        uav2.target = None
                    uav2.state = "Slave"
                    uav2.my_leader = uav1
                    if uav2 not in uav1.my_slave_list:
                        uav1.my_slave_list.append(uav2)
                    if uav2 not in relay_uavs:
                        relay_uavs.append(uav2)

            elif uav1.state == "Leader" and uav2.state == "Free":
                DroneManager._attach_slave(uav2, uav1, relay_uavs)

            elif uav1.state == "Free" and uav2.state == "Leader":
                DroneManager._attach_slave(uav1, uav2, relay_uavs)

        return relay_uavs

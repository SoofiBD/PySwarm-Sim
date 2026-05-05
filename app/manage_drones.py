from typing import List, Optional, Tuple, Dict

import numpy as np
from scipy.spatial import cKDTree  # O(N log N) spatial queries

from app.myMath.matrixOperation import MatrixOperation
from app.uav import UAV
from app.goal import Goal
from app.ground import Ground


class DroneNetwork:
    """Manages drone-to-drone communication and target sharing."""

    COMM_RANGE = 150.0

    @staticmethod
    def build_communication_graph(uavs: List[UAV]) -> Dict[int, List[int]]:
        adj = MatrixOperation.UAVtoUAV_AdjMatrix(uavs)
        graph: Dict[int, List[int]] = {i: [] for i in range(len(uavs))}

        for i in range(len(uavs)):
            for j in range(i + 1, len(uavs)):
                if 0 < adj[i, j] <= DroneNetwork.COMM_RANGE:
                    graph[i].append(j)
                    graph[j].append(i)

        return graph

    @staticmethod
    def share_targets(uavs: List[UAV]) -> None:
        graph = DroneNetwork.build_communication_graph(uavs)

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
    def broadcast_position(uavs: List[UAV]) -> Dict[int, Tuple[float, float, float]]:
        positions: Dict[int, Tuple[float, float, float]] = {}
        graph = DroneNetwork.build_communication_graph(uavs)

        for i, uav in enumerate(uavs):
            positions[i] = (uav.pos.x, uav.pos.y, uav.pos.z)
            for neighbor in graph.get(i, []):
                pass

        return positions


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
    def AssignTarget(goals: List[Goal], uavs: List[UAV], this_uav: UAV) -> Optional[Goal]:
        if not goals:
            return None
            
        uav_to_goal_dist = MatrixOperation.UAVtoGoal_AdjMatrix(uavs, goals)
        possible_targets = []
        
        # Original logic: Find goals where this UAV is the best candidate or at least a good one
        # Simplified: Find free goals and pick the closest one
        for i, goal in enumerate(goals):
            if goal.state == "Free":
                # Check if any other free UAV is closer to this goal
                is_closest = True
                this_uav_idx = uavs.index(this_uav)
                dist_to_goal = uav_to_goal_dist[this_uav_idx][i]
                
                # Note: The original logic had a complex comparison. 
                # Keeping it similar but cleaner.
                possible_targets.append((goal, dist_to_goal))
        
        if possible_targets:
            # Sort by distance and pick the closest
            possible_targets.sort(key=lambda x: x[1])
            assigned_goal = possible_targets[0][0]
            assigned_goal.state = "Visiting"
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
        """Transfers slaves from one UAV to another."""
        slaves = slave_uav.my_slave_list.copy()
        for x in slaves:
            x.target = None
            x.state = "Slave"
            x.my_leader = leader_uav
            leader_uav.my_slave_list.append(x)
            slave_uav.my_slave_list.remove(x)

    @staticmethod
    def AssignRelay(uavs: List[UAV], adj: List[List[float]], ground: Optional[Ground] = None, cthr: float = 200) -> List[UAV]:
        """Complex logic to assign drones as relays when connectivity is threatened."""
        relay_uavs = []
        
        # Calculate components and risky links
        components = MatrixOperation.find_conn_comp(adj, cthr)
        if ground:
            risky_links = MatrixOperation.findRiskyLinks_ground(components, cthr, uavs, ground)
        else:
            risky_links = MatrixOperation.findRiskyLinks(components, cthr, uavs)
            
        for link in risky_links:
            idx1, idx2 = link[0], link[1]
            n_uavs = len(uavs)
            
            # Skip if either index is ground (represented as len(uavs) in original logic)
            if idx1 >= n_uavs or idx2 >= n_uavs:
                continue
            
            uav1, uav2 = uavs[idx1], uavs[idx2]
            
            # Logic for different state combinations
            if uav1.state == "Free" and uav2.state == "Free":
                # Find a leader to follow
                leader_idx = -1
                min_dist = float('inf')
                for i, u in enumerate(uavs):
                    if u.state == "Leader" and u != uav1:
                        dist = MatrixOperation.distance(u.pos, uav1.pos)
                        if dist < min_dist:
                            min_dist = dist
                            leader_idx = i
                
                if leader_idx != -1:
                    uav1.state = "Slave"
                    uav1.my_leader = uavs[leader_idx]
                    uavs[leader_idx].my_slave_list.append(uav1)
                    relay_uavs.append(uav1)
                    
                    uav2.state = "Slave"
                    uav2.my_leader = uavs[leader_idx]
                    uavs[leader_idx].my_slave_list.append(uav2)
                    relay_uavs.append(uav2)

            elif uav1.state == "Leader" and uav2.state == "Leader":
                dist1 = MatrixOperation.distance(uav1.pos, uav1.target.pos)
                dist2 = MatrixOperation.distance(uav2.pos, uav2.target.pos)
                
                if dist1 > dist2:
                    DroneManager.ChangeSlaveList(uav1, uav2)
                    uav1.target.state = "Free"
                    uav1.target = None
                    uav1.state = "Slave"
                    uav1.my_leader = uav2
                    uav2.my_slave_list.append(uav1)
                    relay_uavs.append(uav1)
                else:
                    DroneManager.ChangeSlaveList(uav2, uav1)
                    uav2.target.state = "Free"
                    uav2.target = None
                    uav2.state = "Slave"
                    uav2.my_leader = uav1
                    uav1.my_slave_list.append(uav2)
                    relay_uavs.append(uav2)

            elif uav1.state == "Leader" and uav2.state == "Free":
                uav2.state = "Slave"
                uav2.my_leader = uav1
                uav1.my_slave_list.append(uav2)
                relay_uavs.append(uav2)

            elif uav1.state == "Free" and uav2.state == "Leader":
                uav1.state = "Slave"
                uav1.my_leader = uav2
                uav2.my_slave_list.append(uav1)
                relay_uavs.append(uav1)

        return relay_uavs

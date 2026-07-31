import numpy as np
from typing import List
from app.myMath.vector import Vector
from app.myMath.matrix import Matrix

class MatrixOperation:
    @staticmethod
    def distance_XYZ(X1: float, Y1: float, Z1: float, X2: float, Y2: float, Z2: float) -> float:
        return float(np.linalg.norm(np.array([X1, Y1, Z1]) - np.array([X2, Y2, Z2])))

    @staticmethod
    def distance(pos1: Vector, pos2: Vector) -> float:
        return float(np.linalg.norm(pos1._data - pos2._data))

    @staticmethod
    def UAVtoUAV_AdjMatrix(UAVs: List) -> np.ndarray:
        n = len(UAVs)
        adj = np.zeros((n, n))
        positions = np.array([u.getPos()._data for u in UAVs])
        
        # Optimized vectorized distance calculation
        for i in range(n):
            adj[i] = np.linalg.norm(positions - positions[i], axis=1)
        return adj

    @staticmethod
    def UAVtoGoal_AdjMatrix(UAVs: List, Goals: List) -> np.ndarray:
        n_uavs = len(UAVs)
        n_goals = len(Goals)
        adj = np.zeros((n_uavs, n_goals))
        uav_pos = np.array([u.getPos()._data for u in UAVs])
        goal_pos = np.array([g.getPos()._data for g in Goals])
        
        for i in range(n_uavs):
            adj[i] = np.linalg.norm(goal_pos - uav_pos[i], axis=1)
        return adj

    @staticmethod
    def UAVtoGround_AdjMatrix(UAVs: List, ground) -> np.ndarray:
        n = len(UAVs)
        adj = np.zeros((n + 1, n + 1))
        uav_pos = np.array([u.getPos()._data for u in UAVs])
        ground_pos = ground.getPos()._data
        
        # UAV to UAV
        for i in range(n):
            adj[i, :n] = np.linalg.norm(uav_pos - uav_pos[i], axis=1)
            
        # UAV to Ground
        dist_to_ground = np.linalg.norm(uav_pos - ground_pos, axis=1)
        adj[:n, n] = dist_to_ground
        adj[n, :n] = dist_to_ground
        adj[n, n] = 0
        return adj

    @staticmethod
    def degrees(adj: np.ndarray) -> np.ndarray:
        return np.sum(adj, axis=1)

    @staticmethod
    def find_conn_comp(adj: np.ndarray, cthr: float) -> List[List[int]]:
        """Finds connected components using a more efficient graph traversal."""
        n = len(adj)
        # Convert distance matrix to adjacency matrix based on threshold
        connectivity = (adj <= cthr) & (adj > 0)
        # Add diagonal for self-connectivity
        np.fill_diagonal(connectivity, True)
        
        visited = np.zeros(n, dtype=bool)
        components = []
        
        for i in range(n):
            if not visited[i]:
                component = []
                stack = [i]
                visited[i] = True
                while stack:
                    u = stack.pop()
                    component.append(u)
                    # Find neighbors that haven't been visited
                    neighbors = np.where(connectivity[u] & ~visited)[0]
                    for v in neighbors:
                        visited[v] = True
                        stack.append(v)
                components.append(sorted(component))
        return components

    @staticmethod
    def findRiskyLinks(conn_comp: List[List[int]], cthr: float, UAVs: List) -> List[List[int]]:
        """
        For each pair of disconnected components, return their closest node
        pair if a single relay UAV placed between them could bridge both
        halves (i.e. gap <= 2*cthr). Nodes within the same cthr-connected
        component never qualify here since find_conn_comp already merges
        anything <= cthr — that's why the old `dist <= cthr` check always
        produced an empty list, silently disabling relay assignment.
        """
        risky_links = []
        uav_pos = [u.getPos()._data for u in UAVs]

        for i in range(len(conn_comp)):
            for j in range(i + 1, len(conn_comp)):
                best_pair = None
                best_dist = float('inf')
                for node_i in conn_comp[i]:
                    for node_j in conn_comp[j]:
                        dist = np.linalg.norm(uav_pos[node_i] - uav_pos[node_j])
                        if dist < best_dist:
                            best_dist = dist
                            best_pair = [node_i, node_j]
                if best_pair is not None and best_dist <= 2 * cthr:
                    risky_links.append(best_pair)
        return risky_links

    @staticmethod
    def findRiskyLinks_ground(conn_comp: List[List[int]], cthr: float, UAVs: List, ground) -> List[List[int]]:
        """Ground-aware variant of findRiskyLinks — see that docstring."""
        risky_links = []
        n_uavs = len(UAVs)
        uav_pos = [u.getPos()._data for u in UAVs]
        ground_pos = ground.getPos()._data

        for i in range(len(conn_comp)):
            for j in range(i + 1, len(conn_comp)):
                best_pair = None
                best_dist = float('inf')
                for node_i in conn_comp[i]:
                    for node_j in conn_comp[j]:
                        pos_i = uav_pos[node_i] if node_i < n_uavs else ground_pos
                        pos_j = uav_pos[node_j] if node_j < n_uavs else ground_pos
                        dist = np.linalg.norm(pos_i - pos_j)
                        if dist < best_dist:
                            best_dist = dist
                            best_pair = [node_i, node_j]
                if best_pair is not None and best_dist <= 2 * cthr:
                    risky_links.append(best_pair)
        return risky_links

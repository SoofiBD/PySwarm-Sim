"""
3D A* (A-Star) pathfinder for UAV spatial navigation.

Algorithm overview
------------------
Standard A* over a discretised 3-D grid with 26-connected neighbours
(face + edge + corner).  The heuristic is the admissible Euclidean
distance, so the algorithm is guaranteed to find the optimal (shortest)
path.

Time complexity  : O(V log V)  where V = explored grid nodes
Space complexity : O(V)

Obstacle avoidance
------------------
Dynamic obstacles (other drones) are passed as Vector positions; any grid
node whose centre falls within `obstacle_radius` of an obstacle is skipped.
The check is O(obstacles) per node expansion, which is acceptable for the
drone counts we target (N ≤ 50).  For larger N, swap the per-node check
for a cKDTree lookup.
"""

import heapq
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from app.myMath.vector import Vector


# ---------------------------------------------------------------------------
# Internal node type — stored on the open-set heap
# ---------------------------------------------------------------------------
@dataclass(order=True)
class _Node:
    f: float
    g: float = field(compare=False)
    pos: Tuple[float, float, float] = field(compare=False)
    parent: Optional["_Node"] = field(default=None, compare=False, repr=False)


# ---------------------------------------------------------------------------
# Public pathfinder
# ---------------------------------------------------------------------------
class AStarPathfinder:
    """
    3D grid A* pathfinder.

    Parameters
    ----------
    grid_resolution : float
        Spacing between grid nodes in simulation units (metres).
        Finer grids give smoother paths at higher compute cost.
    bounds : (xmin, xmax, ymin, ymax, zmin, zmax)
        Simulation bounding box.  Nodes outside are never generated.
    obstacle_radius : float
        Clearance radius around each dynamic obstacle (metres).
    """

    def __init__(
        self,
        grid_resolution: float = 40.0,
        bounds: Tuple[float, float, float, float, float, float] = (0, 800, 0, 800, 0, 200),
        obstacle_radius: float = 20.0,
    ) -> None:
        self.res = grid_resolution
        self.xmin, self.xmax, self.ymin, self.ymax, self.zmin, self.zmax = bounds
        self.obstacle_radius = obstacle_radius
        self._obs_r2 = obstacle_radius ** 2
        # Cache valid neighbour offsets (26-connected)
        r = grid_resolution
        self._offsets: List[Tuple[float, float, float]] = [
            (dx, dy, dz)
            for dx in (-r, 0.0, r)
            for dy in (-r, 0.0, r)
            for dz in (-r, 0.0, r)
            if not (dx == 0.0 and dy == 0.0 and dz == 0.0)
        ]
        # Precompute step costs for each offset (straight=r, diagonal=√2r, corner=√3r)
        self._step_costs: List[float] = [
            math.sqrt(dx*dx + dy*dy + dz*dz) for dx, dy, dz in self._offsets
        ]

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def find_path(
        self,
        start: Vector,
        goal: Vector,
        obstacles: Optional[List[Vector]] = None,
    ) -> Optional[List[Vector]]:
        """
        Return a list of Vector waypoints from `start` to `goal`, or
        None if the goal is unreachable.

        `obstacles` are dynamic no-fly positions (e.g., other drones).
        Start/goal are NOT treated as obstacles even if a drone sits there.
        """
        obstacles = obstacles or []
        obs_data = [(o.x, o.y, o.z) for o in obstacles]

        s = self._snap(start)
        g = self._snap(goal)

        if s == g:
            return [goal]

        # open set: min-heap of _Node
        open_heap: List[_Node] = []
        g_score: Dict[Tuple[float, float, float], float] = {s: 0.0}
        in_open: Dict[Tuple[float, float, float], float] = {}  # pos → best f seen
        closed: set = set()

        start_node = _Node(f=self._h(s, g), g=0.0, pos=s)
        heapq.heappush(open_heap, start_node)
        in_open[s] = start_node.f

        while open_heap:
            current = heapq.heappop(open_heap)

            if current.pos in closed:
                continue
            closed.add(current.pos)

            if current.pos == g:
                return self._reconstruct(current)

            cx, cy, cz = current.pos
            for (dx, dy, dz), cost in zip(self._offsets, self._step_costs):
                nx, ny, nz = cx + dx, cy + dy, cz + dz
                nb = (nx, ny, nz)

                if nb in closed:
                    continue
                if not self._in_bounds(nx, ny, nz):
                    continue
                if self._blocked(nx, ny, nz, obs_data):
                    continue

                tentative_g = g_score[current.pos] + cost
                if tentative_g >= g_score.get(nb, math.inf):
                    continue

                g_score[nb] = tentative_g
                f = tentative_g + self._h(nb, g)
                node = _Node(f=f, g=tentative_g, pos=nb, parent=current)
                heapq.heappush(open_heap, node)
                in_open[nb] = f

        return None  # unreachable

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _snap(self, v: Vector) -> Tuple[float, float, float]:
        r = self.res
        return (round(v.x / r) * r, round(v.y / r) * r, round(v.z / r) * r)

    @staticmethod
    def _h(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
        # Euclidean heuristic — admissible for any movement cost ≥ euclidean
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

    def _in_bounds(self, x: float, y: float, z: float) -> bool:
        return (self.xmin <= x <= self.xmax and
                self.ymin <= y <= self.ymax and
                self.zmin <= z <= self.zmax)

    def _blocked(self, x: float, y: float, z: float,
                 obs: List[Tuple[float, float, float]]) -> bool:
        r2 = self._obs_r2
        for ox, oy, oz in obs:
            if (x-ox)**2 + (y-oy)**2 + (z-oz)**2 < r2:
                return True
        return False

    @staticmethod
    def _reconstruct(node: _Node) -> List[Vector]:
        path: List[Vector] = []
        cur: Optional[_Node] = node
        while cur is not None:
            path.append(Vector(*cur.pos))
            cur = cur.parent
        path.reverse()
        return path

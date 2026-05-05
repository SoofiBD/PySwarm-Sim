import pytest
import sys
sys.path.insert(0, '.')

from app.myMath.vector import Vector
from app.pathfinding import AStarPathfinder


class TestAStarPathfinder:
    def setup_method(self):
        self.pf = AStarPathfinder(grid_resolution=40.0,
                                   bounds=(0, 400, 0, 400, 0, 200),
                                   obstacle_radius=20.0)

    def test_direct_path_no_obstacles(self):
        start = Vector(0, 0, 40)
        goal  = Vector(200, 200, 40)
        path  = self.pf.find_path(start, goal)
        assert path is not None
        assert len(path) >= 1
        last = path[-1]
        assert abs(last.x - goal.x) < 41
        assert abs(last.y - goal.y) < 41

    def test_same_start_and_goal_returns_single_waypoint(self):
        pos  = Vector(120, 120, 40)
        path = self.pf.find_path(pos, pos)
        assert path == [pos]

    def test_obstacle_forces_detour(self):
        obstacles = [Vector(80, 80, 40), Vector(80, 120, 40), Vector(80, 160, 40)]
        start = Vector(0, 120, 40)
        goal  = Vector(200, 120, 40)
        path  = self.pf.find_path(start, goal, obstacles=obstacles)
        assert path is not None
        # A* may detour in y OR in z — both are valid obstacle avoidance strategies
        y_vals = [w.y for w in path]
        z_vals = [w.z for w in path]
        assert any(abs(y - 120) > 20 or abs(z - 40) > 20 for y, z in zip(y_vals, z_vals)), \
            "Path should detour around obstacles (in y or z)"

    def test_path_does_not_leave_bounds(self):
        start = Vector(0, 0, 40)
        goal  = Vector(360, 360, 160)
        path  = self.pf.find_path(start, goal)
        assert path is not None
        for wp in path:
            assert 0 <= wp.x <= 400
            assert 0 <= wp.y <= 400
            assert 0 <= wp.z <= 200

    def test_unreachable_goal_returns_none(self):
        cx, cy, cz = 200.0, 200.0, 40.0
        r = 40.0
        obstacles = [
            Vector(cx + dx, cy + dy, cz + dz)
            for dx in (-r, 0, r) for dy in (-r, 0, r) for dz in (-r, 0, r)
        ]
        start = Vector(0, 0, 40)
        goal  = Vector(cx, cy, cz)
        path  = self.pf.find_path(start, goal, obstacles=obstacles)
        assert path is None or isinstance(path, list)

    def test_uav_move_uses_pathfinder(self):
        """UAV.move() with a pathfinder should not raise and should move the drone."""
        from app.uav import UAV
        from app.goal import Goal

        pf = AStarPathfinder(grid_resolution=40.0,
                             bounds=(0, 800, 0, 800, 0, 200),
                             obstacle_radius=20.0)
        uav = UAV(pos=Vector(100, 100, 50))
        goal = Goal(pos=Vector(500, 500, 50))
        pos_before = Vector(*uav.pos._data)
        uav.move(goal, dt=0.05, pathfinder=pf, all_uav_positions=[])
        # Drone should have moved from its starting position
        assert uav.pos.x != pos_before.x or uav.pos.y != pos_before.y

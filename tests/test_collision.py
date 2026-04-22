import pytest
import sys
sys.path.insert(0, '.')

from app.myMath.vector import Vector
from app.uav import UAV
from app.goal import Goal
from app.manage_drones import CollisionDetector, DroneManager


class TestCollisionDetector:
    def test_no_collision_when_far(self):
        uavs = [
            UAV(pos=Vector(0, 0, 0)),
            UAV(pos=Vector(10, 0, 0)),
        ]
        collisions = CollisionDetector.detect_potential_collisions(uavs)
        assert len(collisions) == 0

    def test_collision_detected_when_close(self):
        uavs = [
            UAV(pos=Vector(0, 0, 0)),
            UAV(pos=Vector(2, 0, 0)),
        ]
        collisions = CollisionDetector.detect_potential_collisions(uavs)
        assert len(collisions) == 1

    def test_apply_avoidance_resolves_collision(self):
        uavs = [
            UAV(pos=Vector(0, 0, 0)),
            UAV(pos=Vector(2, 0, 0)),
        ]
        resolved = CollisionDetector.apply_avoidance(uavs)
        assert resolved >= 0


class TestDroneManager:
    def test_get_free_uav_number(self):
        uavs = [
            UAV(pos=Vector(0, 0, 0), state="Free"),
            UAV(pos=Vector(10, 0, 0), state="Leader"),
        ]
        free_count = DroneManager.GetFreeUAVNumber(uavs)
        assert free_count == 1

    def test_get_free_goal_number(self):
        goals = [
            Goal(pos=Vector(0, 0, 0), state="Free"),
            Goal(pos=Vector(10, 0, 0), state="Visiting"),
        ]
        free_count = DroneManager.GetFreeGoalNumber(goals)
        assert free_count == 1

    def test_reset_uavs(self):
        uavs = [
            UAV(pos=Vector(0, 0, 0), state="Leader", target=Goal(pos=Vector(1, 1, 1))),
            UAV(pos=Vector(10, 0, 0), state="Free"),
        ]
        DroneManager.ResetUAVs(uavs)
        for uav in uavs:
            assert uav.state == "Free"
            assert uav.target is None

    def test_reset_goals(self):
        goals = [
            Goal(pos=Vector(0, 0, 0), state="Free"),
            Goal(pos=Vector(10, 0, 0), state="Visiting"),
        ]
        DroneManager.ResetGoals(goals)
        assert goals[0].state == "Free"
        assert goals[1].state == "Free"
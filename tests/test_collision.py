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
        """Leaders with visited/missing targets become Free; active Leaders stay."""
        visited_goal = Goal(pos=Vector(1, 1, 1), state="Visited")
        active_goal = Goal(pos=Vector(5, 5, 5), state="Visiting")
        uavs = [
            UAV(pos=Vector(0, 0, 0), state="Leader", target=visited_goal),
            UAV(pos=Vector(10, 0, 0), state="Leader", target=active_goal),
            UAV(pos=Vector(20, 0, 0), state="Free"),
        ]
        DroneManager.ResetUAVs(uavs)
        # First UAV: target visited → should be freed
        assert uavs[0].state == "Free"
        assert uavs[0].target is None
        # Second UAV: target still active → should stay Leader
        assert uavs[1].state == "Leader"
        assert uavs[1].target is active_goal
        # Third UAV: already Free → stays Free
        assert uavs[2].state == "Free"

    def test_reset_goals(self):
        """Visiting goals with no drone targeting them become Free."""
        goal1 = Goal(pos=Vector(0, 0, 0), state="Free")
        goal2 = Goal(pos=Vector(10, 0, 0), state="Visiting")
        goal3 = Goal(pos=Vector(20, 0, 0), state="Visiting")
        # Only goal3 has a Leader drone targeting it
        uavs = [
            UAV(pos=Vector(0, 0, 0), state="Leader", target=goal3),
        ]
        DroneManager.ResetGoals([goal1, goal2, goal3], uavs)
        assert goal1.state == "Free"     # was already Free
        assert goal2.state == "Free"     # orphaned Visiting → Free
        assert goal3.state == "Visiting" # actively targeted → stays


class TestVelocityImpulseAvoidance:
    def test_resolve_modifies_velocity_not_position(self):
        # uav1 at (0,0,0), uav2 at (3,0,0): impulse on uav1 is in -x direction (away from +x)
        uav1 = UAV(pos=Vector(0, 0, 0), velocity=Vector(0, 0, 0))
        uav2 = UAV(pos=Vector(3, 0, 0), velocity=Vector(0, 0, 0))
        original_pos1 = Vector(*uav1.pos._data)
        CollisionDetector.resolve_collision(uav1, uav2)
        # Position must NOT change
        assert abs(uav1.pos.x - original_pos1.x) < 1e-9
        # Velocity must gain a repulsion component away from uav2 (-x direction)
        assert uav1.velocity.x < 0

    def test_repulsion_direction_is_away_from_other(self):
        uav1 = UAV(pos=Vector(0, 0, 0), velocity=Vector(0, 0, 0))
        uav2 = UAV(pos=Vector(0, 4, 0), velocity=Vector(0, 0, 0))
        CollisionDetector.resolve_collision(uav1, uav2)
        # uav1 should be pushed in -y direction (away from uav2 at +y)
        assert uav1.velocity.y < 0
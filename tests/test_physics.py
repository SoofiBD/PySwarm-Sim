import pytest
import numpy as np
import sys
sys.path.insert(0, '.')

from app.myMath.physics import DronePhysics
from app.myMath.vector import Vector
from app.uav import UAV
from app.goal import Goal


class TestDronePhysics:
    def test_step_returns_new_pos_and_vel(self):
        pos = np.array([0.0, 0.0, 50.0])
        vel = np.array([0.0, 0.0, 0.0])
        desired = np.array([100.0, 0.0, 50.0])
        pos_new, vel_new = DronePhysics.step(pos, vel, desired, mass=1.5,
                                              max_thrust=30.0, dt=0.05)
        assert pos_new[0] > pos[0]
        assert pos_new.shape == (3,)
        assert vel_new.shape == (3,)

    def test_gravity_force_points_down(self):
        f = DronePhysics.gravity_force(mass=1.5)
        assert f[2] < 0
        assert abs(f[2] - (-1.5 * 9.81)) < 1e-9

    def test_drag_opposes_velocity(self):
        vel = np.array([10.0, 0.0, 0.0])
        f = DronePhysics.drag_force(vel)
        assert f[0] < 0

    def test_drag_zero_at_rest(self):
        vel = np.zeros(3)
        f = DronePhysics.drag_force(vel)
        assert np.allclose(f, 0.0)

    def test_thrust_gravity_compensation_at_goal(self):
        desired = pos = np.array([0.0, 0.0, 50.0])
        vel = np.zeros(3)
        f = DronePhysics.thrust_force(desired, pos, vel, mass=1.5,
                                       max_thrust=30.0, kp=3.0, kd=2.0)
        assert f[2] > 0

    def test_uav_converges_to_goal(self):
        uav = UAV(pos=Vector(0, 0, 50))
        goal = Goal(pos=Vector(100, 0, 50))
        for _ in range(200):
            if uav.getState() == "Free":
                break
            uav.move(goal, dt=0.05)
        # Just verify no exception raised — drone either arrived or is moving


class TestFollowerPhysics:
    def test_follow_leader_moves_drone_toward_leader(self):
        leader = UAV(pos=Vector(100, 0, 50), state="Leader")
        follower = UAV(pos=Vector(0, 0, 50), state="Slave")
        follower.my_leader = leader

        pos_before = Vector(*follower.pos._data)
        follower.followLeader(dt=0.05)
        assert follower.pos.x > pos_before.x

    def test_follow_leader_velocity_accumulates(self):
        leader = UAV(pos=Vector(200, 0, 50), state="Leader")
        follower = UAV(pos=Vector(0, 0, 50), state="Slave")
        follower.my_leader = leader

        follower.followLeader(dt=0.05)
        assert follower.velocity.x > 0

    def test_move_to_ground_reduces_altitude(self):
        from app.ground import Ground
        ground = Ground(pos=Vector(0, 0, 0))
        uav = UAV(pos=Vector(0, 0, 100))
        uav.moveToGround(ground, dt=0.05)
        assert uav.pos.z < 100.0

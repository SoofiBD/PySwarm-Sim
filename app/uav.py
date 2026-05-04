"""
UAV domain model.

Physics upgrade
---------------
The previous `move()` kinematically teleported the drone by a fixed number
of units per tick.  The new implementation integrates Newton's 2nd law at
each timestep using `DronePhysics.step()`:

    F_net = F_thrust(PD) + F_gravity + F_drag
    a = F_net / mass
    v(t+dt) = v(t) + a·dt
    x(t+dt) = x(t) + v(t+dt)·dt

Physical parameters (typical 350-class racing quadcopter):
    mass        = 1.5 kg
    max_thrust  = 30 N   (4 × ~7.5 N motors at ~50 % throttle cruise)
"""

from typing import List, Optional

import numpy as np
from pydantic import Field

from app.goal import Goal
from app.myMath.physics import DronePhysics
from app.myMath.vector import Vector, VectorOperations
from app.node import Node


class UAV(Node):
    speed: int = 3
    direction: Vector = Field(default_factory=lambda: Vector(1, 1, 1))
    state: str = "Free"
    ground: Optional["Ground"] = None
    assign_permission: bool = Field(default=True, alias="assignPermission")
    simulation: bool = True
    my_leader: Optional["UAV"] = Field(default=None, alias="myLeader")
    my_slave_list: List["UAV"] = Field(default_factory=list, alias="mySlaveList")
    target: Optional[Goal] = None
    uav_no: Optional[int] = Field(default=None, alias="UAVNo")
    known_targets: List[Goal] = Field(default_factory=list, alias="knownTargets")
    distance_to_target: float = Field(default=0.0, alias="distanceToTarget")

    # ---- Physics state ----
    velocity: Vector = Field(default_factory=lambda: Vector(0.0, 0.0, 0.0))
    mass: float = 1.5       # kg
    max_thrust: float = 30.0  # N

    # ---- Compatibility getters / setters ----
    def getSpeed(self): return self.speed
    def setSpeed(self, v): self.speed = v
    def getState(self): return self.state
    def setState(self, v): self.state = v
    def getTarget(self): return self.target
    def setTarget(self, v): self.target = v
    def getMyLeader(self): return self.my_leader
    def setMyLeader(self, v): self.my_leader = v
    def getMySlaveListComplete(self): return self.my_slave_list
    def getUAVNo(self): return self.uav_no
    def setUAVNo(self, v): self.uav_no = v
    def getKnownTargets(self): return self.known_targets
    def setKnownTargets(self, v): self.known_targets = v
    def addKnownTarget(self, goal): self.known_targets.append(goal)
    def getDistanceToTarget(self): return self.distance_to_target
    def setDistanceToTarget(self, v): self.distance_to_target = v

    def clearMySlaveList(self):
        if self.my_slave_list:
            for x in self.my_slave_list:
                x.setState("Free")
            self.my_slave_list.clear()

    def getLeader(self) -> Optional["UAV"]:
        if self.state == "Leader":
            return self
        elif self.state == "Slave":
            return self.my_leader
        return None

    # ---- Movement ----
    def moveToGround(self, ground: "Ground"):
        if not VectorOperations.isAlmostEqual(self.pos, ground.pos, self.speed):
            self.direction = VectorOperations.substract(ground.pos, self.pos)
            step = VectorOperations.multiply(self.direction.normalize(), self.speed)
            self.pos = VectorOperations.sum(self.pos, step)

    def followLeader(self):
        leader = self.getLeader()
        if leader and leader != self:
            if not VectorOperations.isAlmostEqual(self.pos, leader.pos, self.speed):
                self.direction = VectorOperations.substract(leader.pos, self.pos)
                step = VectorOperations.multiply(self.direction.normalize(), self.speed)
                self.pos = VectorOperations.sum(self.pos, step)

    def move(self, goal: Goal, dt: float = 0.05) -> None:
        """
        Physics-based movement toward `goal`.

        Uses a PD-controller thrust + gravity + quadratic drag model.
        The drone is considered to have arrived when it is within `speed`
        simulation-units of the goal (arrival threshold kept consistent
        with the old kinematic move for state-machine compatibility).
        """
        if VectorOperations.isAlmostEqual(self.pos, goal.pos, float(self.speed)):
            goal.setState("Visited")
            goal.setColor("red")
            self.clearMySlaveList()
            self.setState("Free")
            self.target = None
            self.velocity = Vector(0.0, 0.0, 0.0)
            return

        pos_new_arr, vel_new_arr = DronePhysics.step(
            pos=self.pos._data.copy(),
            vel=self.velocity._data.copy(),
            desired_pos=goal.pos._data.copy(),
            mass=self.mass,
            max_thrust=self.max_thrust,
            dt=dt,
        )
        self.pos = Vector(*pos_new_arr)
        self.velocity = Vector(*vel_new_arr)
        self.direction = VectorOperations.substract(goal.pos, self.pos)


# Resolve forward references
from app.ground import Ground  # noqa: E402
UAV.model_rebuild()

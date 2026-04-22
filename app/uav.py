from typing import List, Optional
from pydantic import Field
from app.node import Node
from app.myMath.vector import Vector, VectorOperations
from app.goal import Goal
import os

class UAV(Node):
    speed: int = 3
    direction: Vector = Field(default_factory=lambda: Vector(1, 1, 1))
    state: str = "Free"
    ground: Optional['Ground'] = None
    assign_permission: bool = Field(default=True, alias="assignPermission")
    simulation: bool = True
    my_leader: Optional['UAV'] = Field(default=None, alias="myLeader")
    my_slave_list: List['UAV'] = Field(default_factory=list, alias="mySlaveList")
    target: Optional[Goal] = None
    uav_no: Optional[int] = Field(default=None, alias="UAVNo")
    known_targets: List[Goal] = Field(default_factory=list, alias="knownTargets")
    distance_to_target: float = Field(default=0.0, alias="distanceToTarget")

    # Compatibility getters/setters (subset)
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

    def getLeader(self) -> Optional['UAV']:
        if self.state == "Leader":
            return self
        elif self.state == "Slave":
            return self.my_leader
        return None

    def moveToGround(self, ground: 'Ground'):
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

    def move(self, goal: Goal):
        if VectorOperations.isAlmostEqual(self.pos, goal.pos, self.speed):
            goal.setState("Visited")
            goal.setColor("red")
            self.clearMySlaveList()
            self.setState("Free")
            self.target = None
        else:
            self.direction = VectorOperations.substract(goal.pos, self.pos)
            step = VectorOperations.multiply(self.direction.normalize(), self.speed)
            self.pos = VectorOperations.sum(self.pos, step)

# Re-import for type hints if needed, but using strings is safer for circularity
from app.ground import Ground
UAV.model_rebuild()

from pydantic import BaseModel, ConfigDict, Field
from typing import Optional
from app.myMath.vector import Vector
from app.persistence import upsert_by_key

class Goal(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)

    pos: Vector = Field(default_factory=lambda: Vector(0, 0, 0))
    goal_no: Optional[int] = Field(default=None, alias="goalNo")
    state: str = "Free"
    color: str = "green"
    age_seconds: float = Field(default=0.0, alias="ageSeconds")

    # Compatibility getters/setters
    def getColor(self): return self.color
    def setColor(self, value): self.color = value
    def getPos(self): return self.pos
    def setPos(self, value): self.pos = value
    def getState(self): return self.state
    def setState(self, value): self.state = value
    def getGoalNo(self): return self.goal_no
    def setGoalNo(self, value): self.goal_no = value

    def toJson(self, filePath: str):
        item_data = {
            "goalNo": self.goal_no,
            "state": self.state,
            "X": self.pos.x,
            "Y": self.pos.y,
            "Z": self.pos.z,
            "color": self.color
        }
        upsert_by_key(filePath, "goalNo", self.goal_no, item_data)

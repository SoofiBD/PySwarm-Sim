from pydantic import BaseModel, ConfigDict, Field
from typing import Optional
from app.myMath.vector import Vector

class Node(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)

    ip_address: Optional[str] = Field(default=None, alias="IPAddress")
    pos: Vector = Field(default_factory=lambda: Vector(0, 0, 0))
    cthr: Optional[float] = Field(default=None)

    def getCthr(self): return self.cthr
    def setCthr(self, value): self.cthr = value
    def getPos(self): return self.pos
    def setPos(self, value): self.pos = value
    def getIPAddress(self): return self.ip_address
    def setIPAddress(self, value): self.ip_address = value

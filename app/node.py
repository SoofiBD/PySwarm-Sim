from pydantic import BaseModel, ConfigDict, Field
from typing import Optional
from app.myMath.vector import Vector
from app.persistence import upsert_by_key

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

    def toJson(self, filePath: str):
        """Standardized JSON serialization via centralized persistence."""
        node_data = {
            "ip": self.ip_address,
            "mode": 0,
            "cthr": self.cthr,
            "X": self.pos.x,
            "Y": self.pos.y,
            "Z": self.pos.z
        }
        upsert_by_key(filePath, "ip", self.ip_address, node_data)

from app.node import Node
from app.myMath.vector import Vector

class Ground(Node):
    # Pydantic handles initialization from fields
    # Default values for Ground
    ip_address: str = "10.0.0.1"
    pos: Vector = Vector(0, 0, 0)

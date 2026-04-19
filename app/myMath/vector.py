import numpy as np
import math

class Vector:
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self._data = np.array([x, y, z], dtype=np.float64)

    @property
    def x(self) -> float:
        return self._data[0]

    @x.setter
    def x(self, value: float):
        self._data[0] = value

    @property
    def y(self) -> float:
        return self._data[1]

    @y.setter
    def y(self, value: float):
        self._data[1] = value

    @property
    def z(self) -> float:
        return self._data[2]

    @z.setter
    def z(self, value: float):
        self._data[2] = value

    # Compatibility getters (matching old codebase)
    def getX(self) -> float: return self.x
    def getY(self) -> float: return self.y
    def getZ(self) -> float: return self.z
    def setX(self, value: float): self.x = value
    def setY(self, value: float): self.y = value
    def setZ(self, value: float): self.z = value

    def get_length(self) -> float:
        return np.linalg.norm(self._data)

    # Compatibility name
    def getLength(self) -> float:
        return self.get_length()

    def normalize(self):
        norm = self.get_length()
        if norm == 0:
            return Vector(0, 0, 0)
        new_data = self._data / norm
        return Vector(*new_data)
    
    def to_string(self) -> str:
        return f"{self.x}:{self.y}:{self.z}"

    # Compatibility name
    def toString(self) -> str:
        return self.to_string()

    def __repr__(self):
        return f"Vector({self.x}, {self.y}, {self.z})"

class VectorOperations:
    @staticmethod
    def sum(v1: Vector, v2: Vector) -> Vector:
        return Vector(*(v1._data + v2._data))

    @staticmethod
    def substract(v1: Vector, v2: Vector) -> Vector:
        return Vector(*(v1._data - v2._data))

    @staticmethod
    def isEqual(v1: Vector, v2: Vector) -> bool:
        return np.array_equal(v1._data, v2._data)

    @staticmethod
    def isAlmostEqual(v1: Vector, v2: Vector, threshold: float) -> bool:
        # Replaced the complex logic with a clean distance-based threshold
        return np.linalg.norm(v1._data - v2._data) <= threshold

    @staticmethod
    def multiply(v1: Vector, scalar: float) -> Vector:
        return Vector(*(v1._data * scalar))

    @staticmethod
    def divide(v1: Vector, scalar: float) -> Vector:
        if scalar == 0:
            raise ValueError("Division by zero in VectorOperations")
        return Vector(*(v1._data / scalar))

    @staticmethod
    def dotProduct(v1: Vector, v2: Vector) -> float:
        return np.dot(v1.normalize()._data, v2.normalize()._data)

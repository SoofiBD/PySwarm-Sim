import math

class Trigonometry:
    def __init__(self) -> None:
        pass

    @staticmethod
    def DegreeToRadian(degree: float) -> float:
        return math.pi * degree / 180.0

    @staticmethod
    def RadianToDegree(radian: float) -> float:
        return radian * (180.0 / math.pi)

    @staticmethod
    def NegativeRadianToDegree(radian: float) -> float:
        degree = radian * (180.0 / math.pi)
        if degree < 0:
            degree = 360 + degree
        return degree

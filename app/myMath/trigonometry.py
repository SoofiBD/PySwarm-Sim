import math

class Trigonometry:
    def __init__(self):
        pass

    @staticmethod
    def DegreeToRadian(degree):
        return math.pi * degree / 180.0

    @staticmethod
    def RadianToDegree(radian):
        return radian * (180.0 / math.pi)

    @staticmethod
    def NegativeRadianToDegree(radian):
        degree = radian * (180.0 / math.pi)
        if degree < 0:
            degree = 360 + degree
        return degree

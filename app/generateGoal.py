from app.goal import Goal
from app.myMath.vector import Vector
import random


class GenerateGoal:

    @staticmethod
    def Run(count:int=None, Goals:list[Goal]=[]):
        count = len(Goals) + count
        i:int = 0
        goalNo:int = 0
        for i in range(len(Goals), count):
            X = random.randrange(0, 800)
            Y = random.randrange(0, 800)
            Z = 0 #random.randrange(0, 800)
            Goals.append(Goal(pos=Vector(X, Y, Z)))
            Goals[len(Goals) - 1].setGoalNo(len(Goals))
            Goals[len(Goals) - 1].toJson("map/static/json/target.json")

        return Goals

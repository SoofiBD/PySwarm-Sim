from app.node import Node
from app.ground import Ground
from app.goal import Goal
from app.uav import UAV
from app.myMath.vector import Vector
from app.myMath.matrixOperation import MatrixOperation
from app.myMath.matrix import Matrix

import random

class GenerateUAV:

    @staticmethod
    def Run(count:int=None, loctype:int=None, cthr:float=None, UAVs:list[UAV]=[], ground:Ground=None):
        count:int = len(UAVs) + count
        i:int = 0        
        UAVno:int = 0
        done:bool = True
        if ground != None:
            done = False
            while (not done):
                X:float = random.randrange(0, 800)
                Y:float = random.randrange(0, 800)
                Z:float = 0
                if MatrixOperation.distance_XYZ(X, Y, Z, ground.getPos().getX(), ground.getPos().getY(), ground.getPos().getZ()) < (cthr * 0.9):
                    done = True
                    UAVs.append(UAV(pos=Vector(X, Y, Z), cthr=cthr, ground=ground))
                    UAVs[len(UAVs) - 1].setUAVNo(len(UAVs))
                    UAVs[len(UAVs) - 1].setIPAddress("10.0.0." + str(len(UAVs)+1))
                    UAVs[len(UAVs) - 1].toJson("map/static/json/drone.json")           
        
        for i in range(len(UAVs), count):
            if (loctype == 1):
                if (i > 0):
                    if (i==1):
                        UAVno = 0
                    else:
                        UAVno = random.randrange(0, i - 1)
                    done = False
                    while(not done):
                        X = random.randrange(0, 800)
                        Y = random.randrange(0, 800)
                        Z = 0
                        if MatrixOperation.distance_XYZ(X, Y, Z, UAVs[UAVno].getPos().getX(), UAVs[UAVno].getPos().getY(), UAVs[UAVno].getPos().getZ()) < (cthr * 0.9):
                            done = True
                            UAVs.append(UAV(pos=Vector(X, Y, Z), cthr=cthr, ground=ground))
                            UAVs[len(UAVs) - 1].setUAVNo(len(UAVs))
                            UAVs[len(UAVs) - 1].setIPAddress("10.0.0." + str(len(UAVs)+1))
                            UAVs[len(UAVs) - 1].toJson("map/static/json/drone.json")
                else:
                    X = random.randrange(0, 800)
                    Y = random.randrange(0, 800)
                    Z = 0
                    UAVs.append(UAV(pos=Vector(X, Y, Z), cthr=cthr, ground=ground))
                    UAVs[len(UAVs) - 1].setUAVNo(len(UAVs))
                    UAVs[len(UAVs) - 1].setIPAddress("10.0.0." + str(len(UAVs)+1))
                    UAVs[len(UAVs) - 1].toJson("map/static/json/drone.json")
            else:
                done = False
                while (not done):
                    X = random.randrange(0, 800)
                    Y = random.randrange(0, 800)
                    Z = 0
                    if MatrixOperation.distance_XYZ(X, Y, Z, UAVs[UAVno].getPos().getX(), UAVs[UAVno].getPos().getY(), UAVs[UAVno].getPos().getZ()) < (cthr * 0.9):
                        done = True
                        UAVs.append(UAV(pos=Vector(X, Y, Z), cthr=cthr, ground=ground))
                        UAVs[len(UAVs) - 1].setUAVNo(len(UAVs))
                        UAVs[len(UAVs) - 1].setIPAddress("10.0.0." + str(len(UAVs)+1))
                        UAVs[len(UAVs) - 1].toJson("map/static/json/drone.json")
        return UAVs

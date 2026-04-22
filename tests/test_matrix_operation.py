import pytest
import sys
sys.path.insert(0, '.')

from app.myMath.vector import Vector
from app.myMath.matrixOperation import MatrixOperation
from app.uav import UAV
from app.goal import Goal


class TestMatrixOperation:
    def test_distance_xyz(self):
        dist = MatrixOperation.distance_XYZ(0, 0, 0, 3, 4, 0)
        assert dist == 5.0

    def test_distance(self):
        v1 = Vector(0, 0, 0)
        v2 = Vector(3, 4, 0)
        dist = MatrixOperation.distance(v1, v2)
        assert dist == 5.0

    def test_uav_to_uav_adj_matrix(self):
        uavs = [
            UAV(pos=Vector(0, 0, 0)),
            UAV(pos=Vector(10, 0, 0)),
        ]
        adj = MatrixOperation.UAVtoUAV_AdjMatrix(uavs)
        assert adj.shape == (2, 2)
        assert adj[0, 1] == 10.0

    def test_uav_to_goal_adj_matrix(self):
        uavs = [UAV(pos=Vector(0, 0, 0))]
        goals = [Goal(pos=Vector(3, 4, 0))]
        adj = MatrixOperation.UAVtoGoal_AdjMatrix(uavs, goals)
        assert adj.shape == (1, 1)
        assert adj[0, 0] == 5.0


class TestConnectivity:
    def test_find_conn_comp_single(self):
        uavs = [
            UAV(pos=Vector(0, 0, 0)),
            UAV(pos=Vector(10, 0, 0)),
            UAV(pos=Vector(20, 0, 0)),
        ]
        adj = MatrixOperation.UAVtoUAV_AdjMatrix(uavs)
        components = MatrixOperation.find_conn_comp(adj, 15.0)
        assert len(components) == 1

    def test_find_conn_comp_multiple(self):
        uavs = [
            UAV(pos=Vector(0, 0, 0)),
            UAV(pos=Vector(100, 0, 0)),
            UAV(pos=Vector(200, 0, 0)),
        ]
        adj = MatrixOperation.UAVtoUAV_AdjMatrix(uavs)
        components = MatrixOperation.find_conn_comp(adj, 15.0)
        assert len(components) == 3
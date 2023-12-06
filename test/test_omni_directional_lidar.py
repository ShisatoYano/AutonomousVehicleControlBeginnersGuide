"""
Unit test of OmniDirectionalLidar

Author: Shisato Yano
"""

import matplotlib as mpl
mpl.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pytest
import sys
from pathlib import Path
from math import sin, cos

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/sensors/lidar")
from scan_point import ScanPoint
from omni_directional_lidar import OmniDirectionalLidar


# mock classes
class MockObstacle:
    def __init__(self):
        pass

    def vertex_xy(self):
        return [25.0, -15.0, -15.0, 15.0, 25.0], [5.0, 5.0, -5.0, -5.0, 5.0]


class MockObstacleList:
    def __init__(self):
        self.list = []
    
    def get_list(self):
        return self.list


class MockXYArray:
    def __init__(self, data):
        self.data = data
    
    def homogeneous_transformation(self, x, y, angle_rad):
        angle_cos = cos(angle_rad)
        angle_sin = sin(angle_rad)

        rotation_matrix = np.array([[angle_cos, -angle_sin],
                                    [angle_sin, angle_cos]])
        
        rotated_data = rotation_matrix @ self.data

        translated_data = rotated_data + np.ones(rotated_data.shape) * np.array([[x], [y]])

        return MockXYArray(translated_data)
    
    def get_x_data(self):
        return self.data[0, :]
    
    def get_y_data(self):
        return self.data[1, :]


class MockSensorParameters:
    def __init__(self):
        self.INST_LON_M = 0.0
        self.INST_LAT_M = 0.0

        self.MIN_RANGE_M = 0.5
        self.MAX_RANGE_M = 40

        self.RESO_RAD = np.deg2rad(2.0)

        self.ANGLE_STD_SCALE = 0.0
        self.DIST_STD_RATE = 0.0

        self.inst_pos_array = MockXYArray(np.array([[self.INST_LON_M], [self.INST_LAT_M]]))
        self.global_x_m = None
        self.global_y_m = None
    
    def calculate_global_pos(self, state):
        pose = state.x_y_yaw()
        transformed_array = self.inst_pos_array.homogeneous_transformation(pose[0, 0], pose[1, 0], pose[2, 0])
        self.global_x_m = transformed_array.get_x_data()
        self.global_y_m = transformed_array.get_y_data()
    
    def calculate_sensor_odometry(self, state):
        pass
    
    def calibrate_extrinsic_params(self, state):
        pass
    
    def get_global_x_m(self):
        return self.global_x_m[0]
    
    def get_global_y_m(self):
        return self.global_y_m[0]
    
    def draw_pos(self, axes, elems, state):
        pos_plot, = axes.plot(self.global_x_m, self.global_y_m, marker='.', color='b')
        elems.append(pos_plot)


class MockState:
    def __init__(self):
        pass
    
    def x_y_yaw(self):
        return np.array([[0.0], [0.0], [0.0]])
    
    def get_yaw_rad(self):
        return 0.0


# test instance
obst = MockObstacle()
obst_list = MockObstacleList()
obst_list.list.append(obst)

params = MockSensorParameters()

state = MockState()

lidar = OmniDirectionalLidar(obst_list, params)


def test_initialize():
    assert len(lidar.obst_list.get_list()) == 1
    assert lidar.params != None
    assert lidar.DIST_DB_SIZE == 181
    assert lidar.MAX_DB_VALUE == float("inf")
    assert lidar.DELTA_LIST[0] == 0.0
    assert lidar.DELTA_LIST[-1] == 0.992
    assert len(lidar.latest_point_cloud) == 0


def test_update():
    lidar.update(state)
    latest_point_cloud = lidar.get_point_cloud()

    assert len(latest_point_cloud) == 162
    assert type(latest_point_cloud[0]) == ScanPoint


def test_draw():
    plt.clf()
    plt.close()
    
    figure = plt.figure(figsize=(8, 8))
    axes = figure.add_subplot(111)

    lidar.draw(axes, [], state)

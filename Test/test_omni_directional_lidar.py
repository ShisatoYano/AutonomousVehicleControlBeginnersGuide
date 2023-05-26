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

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/sensors/lidar")
from omni_directional_lidar import OmniDirectionalLidar


# mock classes
class MockObstacle:
    def __init__(self):
        pass

    def vertex_xy(self):
        return [0.0, 1.0, 2.0, 3.0, 4.0], [0.0, 1.0, 2.0, 3.0, 4.0]


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
    
    def get_global_x_m(self):
        return self.global_x_m[0]
    
    def get_global_y_m(self):
        return self.global_y_m[0]
    
    def draw_pos(self, axes, elems):
        pos_plot, = axes.plot(self.global_x_m, self.global_y_m, marker='.', color='b')
        elems.append(pos_plot)


class MockState:
    def __init__(self):
        pass
    
    def x_y_yaw(self):
        return np.array([[0.0], [0.0], [0.0]])
    
    def get_yaw_rad(self):
        return 0.0

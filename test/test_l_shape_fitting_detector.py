"""
Unit test of PurePursuitController

Author: Shisato Yano
"""

import matplotlib as mpl
mpl.use("Agg")
import matplotlib.pyplot as plt
from math import sin, cos
import numpy as np
import pytest
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/detection/l_shape_fitting")
from l_shape_fitting_detector import LShapeFittingDetector


# mock class
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


class MockScanPoint:
    def __init__(self, distance_m, angle_rad, x_m, y_m):
        self.distance_m = distance_m
        self.angle_rad = angle_rad
        self.point_array = MockXYArray(np.array([[x_m], [y_m]]))

    def get_distance_m(self):
        return self.distance_m
    
    def get_point_array(self):
        return self.point_array.get_data()
    
    def draw(self, axes, x_m, y_m, angle_rad, elems):
        transformed_array = self.point_array.homogeneous_transformation(x_m, y_m, angle_rad)
        point_plot, = axes.plot(transformed_array.get_x_data(), 
                                transformed_array.get_y_data(), 
                                marker='.', color='b')
        elems.append(point_plot)


# test target instance
detector = LShapeFittingDetector()


def test_initialize():
    assert detector.MIN_RNG_TH_M == 3.0
    assert detector.RNG_TH_RATE == 0.1
    assert detector.CHANGE_ANGLE_RAD == np.deg2rad(1.0)
    assert len(detector.latest_rectangles_list) == 0


def test_update():
    # dummy point cloud
    point_cloud = [MockScanPoint(5.0, 0.0, 5.0, 0.0), MockScanPoint(5.0, 0.1, 5.0, 0.1), MockScanPoint(5.0, -0.1, 5.0, -0.1),
                   MockScanPoint(15.0, 0.0, 15.0, 0.0), MockScanPoint(15.0, 0.1, 15.0, 0.1), MockScanPoint(15.0, -0.1, 15.0, -0.1),
                   MockScanPoint(30.0, 0.0, 30.0, 0.0), MockScanPoint(30.0, 0.1, 30.0, 0.1), MockScanPoint(30.0, -0.1, 30.0, -0.1)]

    detector.update(point_cloud)

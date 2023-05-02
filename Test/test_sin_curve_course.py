"""
Unit test of SinCurveCourse

Author: Shisato Yano
"""

import matplotlib as mpl
mpl.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from math import sin
import pytest
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/course/sin_curve_course")
from sin_curve_course import SinCurveCourse

# mock class
class MockState:
    def __init__(self, x_m, y_m, yaw_rad, speed_mps):
        self.x_m = x_m
        self.y_m = y_m
        self.yaw_rad = yaw_rad
        self.speed_mps = speed_mps
    
    def get_x_m(self):
        return self.x_m

    def get_y_m(self):
        return self.y_m
    
    def get_yaw_rad(self):
        return self.yaw_rad

    def get_speed_mps(self):
        return self.speed_mps


# test parameters
X_MIN = 0
X_MAX = 50
RESOLUTION = 1
TARGET_SPEED_KMPH = 20
INIT_INDEX = 0
MID_INDEX = 24
END_INDEX = 49


# test data
x_array = np.arange(X_MIN, X_MAX, RESOLUTION)
y_array = [sin(x / 5.0) * (x / 2.0) for x in x_array]
speed_array = [(TARGET_SPEED_KMPH / 3.6) for _ in x_array]
speed_array[-1] = 0.0


# mock instances for test
state_x_init = MockState(x_array[INIT_INDEX], y_array[INIT_INDEX], 0.0, speed_array[INIT_INDEX])
state_x_mid = MockState(x_array[MID_INDEX], y_array[MID_INDEX], 0.0, speed_array[MID_INDEX])
state_x_end = MockState(x_array[END_INDEX], y_array[END_INDEX], 0.0, speed_array[END_INDEX])


# test instance
course = SinCurveCourse(X_MIN, X_MAX, RESOLUTION, TARGET_SPEED_KMPH)


def test_attributes():
    assert hasattr(course, "x_array") == True
    assert hasattr(course, "y_array") == True
    assert hasattr(course, "speed_array") == True
    assert len(course.x_array) == 50
    assert len(course.y_array) == 50
    assert len(course.speed_array) == 50
    assert course.speed_array[-1] == 0.0


def test_search_nearest_point_index():
    assert course.search_nearest_point_index(state_x_init) == 0
    assert course.search_nearest_point_index(state_x_mid) == 24
    assert course.search_nearest_point_index(state_x_end) == 49


def test_calculate_distance_from_point():
    assert course.calculate_distance_from_point(state_x_init, INIT_INDEX) == 0.0
    assert course.calculate_distance_from_point(state_x_mid, MID_INDEX) == 0.0
    assert course.calculate_distance_from_point(state_x_end, END_INDEX) == 0.0


def test_calculate_speed_difference_mps():
    assert course.calculate_speed_difference_mps(state_x_init, INIT_INDEX) == 0.0
    assert course.calculate_speed_difference_mps(state_x_mid, MID_INDEX) == 0.0
    assert course.calculate_speed_difference_mps(state_x_end, END_INDEX) == 0.0


def test_calculate_angle_difference_rad():
    assert course.calculate_angle_difference_rad(state_x_init, INIT_INDEX) == 0.0
    assert course.calculate_angle_difference_rad(state_x_mid, MID_INDEX) == 0.0
    assert course.calculate_angle_difference_rad(state_x_end, END_INDEX) == 0.0


def test_point_xy_m():
    assert round(course.point_x_m(INIT_INDEX), 1) == 0.0
    assert round(course.point_y_m(INIT_INDEX), 1) == 0.0

    assert round(course.point_x_m(MID_INDEX), 1) == 24.0
    assert round(course.point_y_m(MID_INDEX), 1) == -12.0

    assert round(course.point_x_m(END_INDEX), 1) == 49.0
    assert round(course.point_y_m(END_INDEX), 1) == -9.0


def test_target_speed_mps():
    assert round(course.target_speed_mps(INIT_INDEX), 1) == 5.6
    assert round(course.target_speed_mps(MID_INDEX), 1) == 5.6
    assert round(course.target_speed_mps(END_INDEX), 1) == 0.0


def test_length():
    assert course.length() == 50


def test_draw():
    figure = plt.figure(figsize=(8, 8))
    axes = figure.add_subplot(111)
    course.draw(axes, [])

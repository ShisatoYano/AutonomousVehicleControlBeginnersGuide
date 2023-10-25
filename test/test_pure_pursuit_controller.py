"""
Unit test of PurePursuitController

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

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/control/pure_pursuit")
from pure_pursuit_controller import PurePursuitController

# mock class
class MockVehicleSpecification:
    def __init__(self):
        self.wheel_base_m = 2.0


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


class MockCourse:
    def __init__(self):
        pass

    def search_nearest_point_index(self, state):
        return 0
    
    def calculate_distance_from_point(self, state, point_index):
        return 5.0
    
    def calculate_speed_difference_mps(self, state, point_index):
        return 1.0
    
    def calculate_angle_difference_rad(self, state, point_index):
        return 2.0
    
    def point_x_m(self, point_index):
        return 3.0
    
    def point_y_m(self, point_index):
        return 4.0
    
    def length(self):
        return 10


# mock instance
spec = MockVehicleSpecification()
course = MockCourse()


def test_without_course_data():
    controller = PurePursuitController(spec)
    state = MockState(0.0, 0.0, 0.0, 0.0)

    assert controller.MIN_LOOK_AHEAD_DISTANCE_M == 2.0
    assert controller.LOOK_FORWARD_GAIN == 0.3
    assert controller.SPEED_PROPORTIONAL_GAIN == 1.0
    assert controller.WHEEL_BASE_M == spec.wheel_base_m
    assert controller.DRAW_COLOR == 'g'

    assert controller.course == None
    assert controller.look_ahead_distance_m == 2.0
    assert controller.target_course_index == 0

    assert controller.update(state) == None
    assert controller.get_target_accel_mps2() == 0.0
    assert controller.get_target_steer_rad() == 0.0
    assert controller.get_target_yaw_rate_rps() == 0.0


def test_with_course_data():
    controller = PurePursuitController(spec, course)
    state = MockState(0.0, 1.0, 2.0, 3.0)

    controller.update(state)

    assert round(controller.get_target_accel_mps2(), 1) == 1.0
    assert round(controller.get_target_steer_rad(), 1) == 0.9
    assert round(controller.get_target_yaw_rate_rps(), 1) == 1.9


def test_draw():
    controller = PurePursuitController(spec, course)

    plt.clf()
    plt.close()
    
    figure = plt.figure(figsize=(8, 8))
    axes = figure.add_subplot(111)
    controller.draw(axes, [])

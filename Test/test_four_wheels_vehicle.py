"""
Unit test of FourWheelsVehicle

Author: Shisato Yano
"""

import matplotlib as mpl
mpl.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pytest
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../src/vehicle")
from four_wheels_vehicle import FourWheelsVehicle


# mock class
class MockSpecification:
    def __init__(self):
        self.f_len_m = 2.0
        self.f_edge_m = self.f_len_m + 0.5

        self.r_len_m = 0.0
        self.r_edge_m = self.r_len_m + 0.5

        self.tread_m = 0.25 * (1.0 + self.f_len_m + self.r_len_m)
        self.width_m = 1.0 * self.tread_m
        self.wheel_base_m = self.f_len_m + self.r_len_m

        self.tire_r_m = 0.3
        self.tire_w_m = 0.12
        self.axle_half_m = 0.5
        
        self.color = 'k'
        self.line_w = 1.0
        self.line_type = '-'


class MockState:
    def __init__(self):
        pass

    def update(self, accel_mps2, steer_rad, time_s, wheel_base_m):
        return MockState()
    
    def x_y_yaw(self):
        return np.array([[0.0], [0.0], [0.0]])
    
    def get_x_m(self):
        return 0.0

    def get_y_m(self):
        return 0.0

class MockStateHistory:
    def __init__(self):
        pass

    def update(self, x, y):
        return MockStateHistory()
    
    def draw(self, axes, color):
        return axes.plot([], [], color=color)


# test instance
spec = MockSpecification()
state = MockState()
history = MockStateHistory()
vehicle = FourWheelsVehicle(state, history, spec)


def test_attributes():
    assert hasattr(vehicle, "state") == True
    assert hasattr(vehicle, "history") == True
    assert hasattr(vehicle, "spec") == True
    assert hasattr(vehicle, "body") == True
    assert hasattr(vehicle, "chassis") == True
    assert hasattr(vehicle, "front_left_tire") == True
    assert hasattr(vehicle, "front_right_tire") == True
    assert hasattr(vehicle, "rear_left_tire") == True
    assert hasattr(vehicle, "rear_right_tire") == True
    assert hasattr(vehicle, "front_axle") == True
    assert hasattr(vehicle, "rear_axle") == True
    assert hasattr(vehicle, "width") == True


def test_initialize():
    assert vehicle.state != None
    assert vehicle.history != None
    assert vehicle.spec != None
    assert vehicle.body != None
    assert vehicle.chassis != None
    assert vehicle.front_left_tire != None
    assert vehicle.front_right_tire != None
    assert vehicle.rear_left_tire != None
    assert vehicle.rear_right_tire != None
    assert vehicle.front_axle != None
    assert vehicle.rear_axle != None
    assert vehicle.width == 10.0


def test_one_step():
    vehicle.one_step(1.0)


def test_draw():
    figure = plt.figure(figsize=(8, 8))
    axes = figure.add_subplot(111)
    vehicle.draw(axes, [])

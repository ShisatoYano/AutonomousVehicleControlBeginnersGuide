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
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/vehicle")
from four_wheels_vehicle import FourWheelsVehicle


# mock class
class MockMinMax():
    def __init__(self):
        pass

    def min_value(self):
        return -10.0
    
    def max_value(self):
        return 10.0


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
        self.area_size = 10.0

        self.x_lim = MockMinMax()
        self.y_lim = MockMinMax()


class MockState:
    def __init__(self):
        pass

    def update(self, accel_mps2, yaw_rate_rps, time_s):
        pass
    
    def x_y_yaw(self):
        return np.array([[0.0], [0.0], [0.0]])
    
    def get_x_m(self):
        return 0.0

    def get_y_m(self):
        return 0.0
    
    def get_speed_kmph(self):
        return 0.0
    
    def draw(self, axes, elems):
        pass


# test instance
spec = MockSpecification()
state = MockState()
vehicle = FourWheelsVehicle(state, spec)


def test_attributes():
    assert hasattr(vehicle, "state") == True
    assert hasattr(vehicle, "spec") == True
    assert hasattr(vehicle, "body") == True
    assert hasattr(vehicle, "chassis") == True
    assert hasattr(vehicle, "front_left_tire") == True
    assert hasattr(vehicle, "front_right_tire") == True
    assert hasattr(vehicle, "rear_left_tire") == True
    assert hasattr(vehicle, "rear_right_tire") == True
    assert hasattr(vehicle, "front_axle") == True
    assert hasattr(vehicle, "rear_axle") == True
    assert hasattr(vehicle, "controller") == True
    assert hasattr(vehicle, "sensors") == True
    assert hasattr(vehicle, "detector") == True
    assert hasattr(vehicle, "localizer") == True
    assert hasattr(vehicle, "show_zoom") == True


def test_initialize():
    assert vehicle.state != None
    assert vehicle.spec != None
    assert vehicle.body != None
    assert vehicle.chassis != None
    assert vehicle.front_left_tire != None
    assert vehicle.front_right_tire != None
    assert vehicle.rear_left_tire != None
    assert vehicle.rear_right_tire != None
    assert vehicle.front_axle != None
    assert vehicle.rear_axle != None
    assert vehicle.controller == None
    assert vehicle.sensors == None
    assert vehicle.detector == None
    assert vehicle.localizer == None
    assert vehicle.show_zoom == True


def test_update():
    vehicle.update(1.0)


def test_draw():
    plt.clf()
    plt.close()
    figure = plt.figure(figsize=(8, 8))
    axes = figure.add_subplot(111)
    vehicle.draw(axes, [])


def test_show_zoom():
    vehicle_show_zoom = FourWheelsVehicle(state, spec, show_zoom=False)
    assert vehicle_show_zoom.show_zoom == False
    test_draw()

"""
Unit test of FourWheelsVehicle

Author: Shisato Yano
"""

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
        self.r_len_m = 0.0
        self.wheel_base_m = self.f_len_m + self.r_len_m
        self.tire_r_m = 0.3
        self.tire_w_m = 0.12
        self.axle_half_m = 0.5
        self.color = 'k'
        self.line_w = 1.0
        self.line_type = '-'


# test instance
spec = MockSpecification()
vehicle = FourWheelsVehicle(np.array([[0.0], [0.0], [0.0]]), spec)


def test_attributes():
    assert hasattr(vehicle, "pose") == True
    assert hasattr(vehicle, "steer_rad") == True
    assert hasattr(vehicle, "spec") == True
    assert hasattr(vehicle, "body") == True
    assert hasattr(vehicle, "chassis") == True
    assert hasattr(vehicle, "front_left_tire") == True
    assert hasattr(vehicle, "front_right_tire") == True
    assert hasattr(vehicle, "rear_left_tire") == True
    assert hasattr(vehicle, "rear_right_tire") == True
    assert hasattr(vehicle, "front_axle") == True
    assert hasattr(vehicle, "rear_axle") == True
    assert hasattr(vehicle, "agent") == True
    assert hasattr(vehicle, "motion") == True
    assert hasattr(vehicle, "poses") == True

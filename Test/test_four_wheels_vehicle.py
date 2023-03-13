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


class MockAgent:
    def __init__(self):
        self.speed_mps =  0.0
        self.yaw_rate_rps = 0.0

    def control_order(self):
        return np.array([[self.speed_mps], [self.yaw_rate_rps]])


class MockMotionModel:
    def __init__(self):
        pass
    
    def state_transition(self, pose, order, time_interval_s):
        return np.array([[0.0], [0.0], [0.0]])
    
    def steering_angle_rad(self, order):
        return 0.0


# test instance
spec = MockSpecification()
pose = np.array([[0.0], [0.0], [0.0]])
vehicle = FourWheelsVehicle(pose, spec)


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


def test_initialize():
    assert vehicle.pose[0, 0] == pose[0, 0]
    assert vehicle.pose[1, 0] == pose[1, 0]
    assert vehicle.pose[2, 0] == pose[2, 0]
    assert vehicle.steer_rad == 0.0
    assert vehicle.spec != None
    assert vehicle.body != None
    assert vehicle.chassis != None
    assert vehicle.front_left_tire != None
    assert vehicle.front_right_tire != None
    assert vehicle.rear_left_tire != None
    assert vehicle.rear_right_tire != None
    assert vehicle.front_axle != None
    assert vehicle.rear_axle != None
    assert vehicle.agent == None
    assert vehicle.motion == None
    assert len(vehicle.poses) == 1


def test_set_agent_motion():
    agent = MockAgent()
    motion = MockMotionModel()
    vehicle = FourWheelsVehicle(pose, spec, agent=agent, motion=motion)

    assert vehicle.agent != None
    assert vehicle.motion != None


def test_one_step():
    vehicle.one_step(1.0)


def test_draw():
    figure = plt.figure(figsize=(8, 8))
    axes = figure.add_subplot(111)
    vehicle.draw(axes, [])

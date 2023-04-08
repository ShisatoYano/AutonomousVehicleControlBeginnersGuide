"""
state.py

Author: Shisato Yano
"""

from math import cos, sin, tan
import numpy as np


class State:
    """
    Vehicle state(x, y, yaw, speed) data and logic class
    """

    def __init__(self, x_m, y_m, yaw_rad, speed_mps):
        """
        Constructor
        x_m: Vehicle's position, x[m]
        y_m: Vehicle's position, y[m]
        yaw_rad: Vehicle's yaw angle[rad]
        speed_mps: Vehicle's speed[m/s]
        """

        self.x_m = x_m
        self.y_m = y_m
        self.yaw_rad = yaw_rad
        self.speed_mps = speed_mps
    
    def update(self, accel_mps2, yaw_rate_rps, time_s):
        """
        Function to update state
        accel_mps2: Acceleration[m/s^2]
        steer_rad: Steering angle[rad]
        time_s: Time interval per cycle[sec]
        wheel_base_m: Wheel base's length[m]
        """
        updated_x_m = self.x_m + self.speed_mps * cos(self.yaw_rad) * time_s
        updated_y_m = self.y_m + self.speed_mps * sin(self.yaw_rad) * time_s

        # yaw_rate_rad = self.speed_mps * tan(steer_rad) / wheel_base_m
        updated_yaw_rad = self.yaw_rad + yaw_rate_rps * time_s

        updated_speed_mps = self.speed_mps + accel_mps2 * time_s

        return State(updated_x_m, updated_y_m, updated_yaw_rad, updated_speed_mps)
    
    def x_y_yaw(self):
        """
        Function to get x, y, yaw as array
        """
        
        return np.array([[self.x_m], [self.y_m], [self.yaw_rad]])
    
    def get_x_m(self):
        """
        Function to get x[m]
        """
        
        return self.x_m
    
    def get_y_m(self):
        """
        Function to get y[m]
        """

        return self.y_m
    
    def get_yaw_rad(self):
        """
        Function to get yaw angle[rad]
        """

        return self.yaw_rad
    
    def get_speed_mps(self):
        """
        Function to get speed[m/s]
        """

        return self.speed_mps

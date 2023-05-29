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

    def __init__(self, x_m=0.0, y_m=0.0, yaw_rad=0.0, speed_mps=0.0, color='k'):
        """
        Constructor
        x_m: Vehicle's position, x[m]
        y_m: Vehicle's position, y[m]
        yaw_rad: Vehicle's yaw angle[rad]
        speed_mps: Vehicle's speed[m/s]
        """

        self.STOP_SPEED_MPS = 0.5 / 3.6 # 0.5[km/h]
        self.MAX_SPEED_MPS = 60 / 3.6 # 60[km/h]
        self.MIN_SPEED_MPS = -10 / 3.6 # -10[km/h]
        self.DRAW_COLOR = color

        self.x_m = x_m
        self.y_m = y_m
        self.yaw_rad = yaw_rad
        self.speed_mps = speed_mps

        self.x_history = [self.x_m]
        self.y_history = [self.y_m]
    
    def update(self, accel_mps2, yaw_rate_rps, time_s):
        """
        Function to update state
        accel_mps2: Acceleration[m/s^2]
        steer_rad: Steering angle[rad]
        time_s: Time interval per cycle[sec]
        """
        self.x_m += self.speed_mps * cos(self.yaw_rad) * time_s
        self.y_m += self.speed_mps * sin(self.yaw_rad) * time_s
        self.yaw_rad += yaw_rate_rps * time_s
        self.speed_mps += accel_mps2 * time_s
        
        if abs(self.speed_mps) < self.STOP_SPEED_MPS: self.speed_mps = 0.0
        if self.speed_mps > self.MAX_SPEED_MPS: self.speed_mps = self.MAX_SPEED_MPS
        if self.speed_mps < self.MIN_SPEED_MPS: self.speed_mps = self.MIN_SPEED_MPS

        self.x_history.append(self.x_m)
        self.y_history.append(self.y_m)
    
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
    
    def get_speed_kmph(self):
        """
        Function to get speed[km/h]
        """

        return self.speed_mps * 3.6
    
    def draw(self, axes, elems):
        """
        Function to draw x-y history and speed
        """
        
        hist_plot, = axes.plot(self.x_history, self.y_history, linewidth=0, marker='.', color=self.DRAW_COLOR)
        elems.append(hist_plot)

        elems.append(axes.text(self.x_m, self.y_m + 2, "Speed: " + str(round(self.speed_mps * 3.6, 1)) + "[km/h]", fontsize=10))

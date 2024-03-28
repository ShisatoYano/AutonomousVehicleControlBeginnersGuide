"""
state.py

Author: Shisato Yano
"""

from math import cos, sin
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
    
    @staticmethod
    def motion_model(state, input, time_s):
        """
        Static function of motion model of vehicle state
        state: Vehicle's state (x, y, yaw, speed) object
        input: Motion input (acceleration, yaw rate) object
        time_s: Time interval per cycle[sec]
        """

        # to fix DeprecationWarning: Conversion of an array with ndim > 0 
        # to a scalar is deprecated, and will error in future. 
        # Ensure you extract a single element from your array 
        # before performing this operation. (Deprecated NumPy 1.25.)
        yaw_rad = state.item(2) # do not extract an element like state[2]
        
        A = np.array([[1, 0, 0, cos(yaw_rad) * time_s],
                      [0, 1, 0, sin(yaw_rad) * time_s],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        
        B = np.array([[(cos(yaw_rad) * time_s**2) / 2, 0],
                      [(sin(yaw_rad) * time_s**2) / 2, 0],
                      [0, time_s],
                      [time_s, 0]])
        
        return A @ state + B @ input

    def update(self, accel_mps2, yaw_rate_rps, time_s):
        """
        Function to update state
        accel_mps2: Acceleration[m/s^2]
        steer_rad: Steering angle[rad]
        time_s: Time interval per cycle[sec]
        """

        last_state = np.array([[self.x_m],
                               [self.y_m],
                               [self.yaw_rad],
                               [self.speed_mps]])
        
        next_input = np.array([[accel_mps2],
                               [yaw_rate_rps]])
        
        next_state = self.motion_model(last_state, next_input, time_s)

        self.x_m = next_state[0, 0]
        self.y_m = next_state[1, 0]
        self.yaw_rad = next_state[2, 0]
        self.speed_mps = next_state[3, 0]
        
        if abs(self.speed_mps) < self.STOP_SPEED_MPS: self.speed_mps = 0.0
        if self.speed_mps > self.MAX_SPEED_MPS: self.speed_mps = self.MAX_SPEED_MPS
        if self.speed_mps < self.MIN_SPEED_MPS: self.speed_mps = self.MIN_SPEED_MPS

        self.x_history.append(self.x_m)
        self.y_history.append(self.y_m)
    
    def update_by_localizer(self, state_from_localizer):
        self.x_m = state_from_localizer[0, 0]
        self.y_m = state_from_localizer[1, 0]
        self.yaw_rad = state_from_localizer[2, 0]
        self.speed_mps = state_from_localizer[3, 0]

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

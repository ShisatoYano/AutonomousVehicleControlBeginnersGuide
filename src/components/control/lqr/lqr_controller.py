"""
lqr_controller.py

Author: Shisato Yano
"""

#import path setting
import sys
from pathlib import Path
from math import tan, atan2
import numpy as np
import scipy.linalg as la

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"

sys.path.append(abs_dir_path + relative_path + "common")

#import component modules
from angle_lib import pi_to_pi


class LqrController:
    """
    Controller class by LQR(Linear Quadratic Regulator) algorithm
    """

    def __init__(self, spec, course=None):
        """
        Constructor
        spec: VehicleSpecification object
        course: Course data and logic object
        """

        self.WHEEL_BASE_M = spec.wheel_base_m

        self.SCALE_R = 4.0
        self.WEIGHT_MAT_Q = np.eye(5)
        self.WEIGHT_MAT_R = self.SCALE_R * np.eye(2)
        self.MAX_ITERATION = 150
        self.THRESHOLD = 0.01

        self.course = course
        self.target_course_index = 0
        self.target_accel_mps2 = 0.0
        self.target_speed_mps = 0.0
        self.target_yaw_rate_rps = 0.0
        self.target_steer_rad = 0.0

        self.prev_error_lat_m = 0.0
        self.prev_error_yaw_rad = 0.0

    def _calculate_target_course_index(self, state):
        """
        Private function to calculate target point's index on course
        state: Vehicle's state object
        """
        
        nearest_index = self.course.search_nearest_point_index(state)
        self.target_course_index = nearest_index

    def _calculate_tracking_error(self, state):
        """
        Private function to calculate tracking error against target point on the course
        state: Vehicle's state object
        """

        error_lon_m, error_lat_m, error_yaw_rad = self.course.calculate_lonlat_error(state, self.target_course_index)
        return error_lon_m, error_lat_m, error_yaw_rad

    def _decide_target_speed_mps(self):
        """
        Private function to decide target speed[m/s]
        """

        self.target_speed_mps = self.course.point_speed_mps(self.target_course_index)

    def _solve_riccati_equation(self, A, B):
        """
        Private function to solve discrete algebraic riccati equation
        x[t+1] = A x[t] + B u[t]
        A: Matrix A in state equation
        B: Matrix B in state equation
        """

        x = self.WEIGHT_MAT_Q
        x_next = self.WEIGHT_MAT_Q

        for i in range(self.MAX_ITERATION):
            x_next = A.T @ x @ A - A.T @ x @ B @ \
                     la.inv(self.WEIGHT_MAT_R + B.T @ x @ B) @ \
                     B.T @ x @ A + self.WEIGHT_MAT_Q
            
            if (abs(x_next - x)).max() < self.THRESHOLD:
                break

            x = x_next
        
        return x_next

    def _calculate_control_gain(self, A, B):
        """
        Private function to calculate control gain
        cost = sum(x[t].T * Q * x[t] + u[t].T * R * u[t])
        A: Matrix A in state equation
        B: Matrix B in state equation
        """

        X = self._solve_riccati_equation(A, B)

        gain = la.inv(B.T @ X @ B + self.WEIGHT_MAT_R) @ (B.T @ X @ A)

        return gain

    def _calculate_control_input(self, state, error_lat_m, error_yaw_rad, time_s):
        """
        Private function to calculate yaw rate input
        state: Vehicle's state object
        error_lat_m: Lateral error against reference course[m]
        error_yaw_rad: Yaw angle error against reference course[rad]
        time_s: Simulation interval time[sec]
        """

        curr_spd = state.get_speed_mps()
        trgt_curv = self.course.point_curvature(self.target_course_index)

        # A = [1.0, dt, 0.0, 0.0, 0.0
        #      0.0, 0.0, v, 0.0, 0.0]
        #      0.0, 0.0, 1.0, dt, 0.0]
        #      0.0, 0.0, 0.0, 0.0, 0.0]
        #      0.0, 0.0, 0.0, 0.0, 1.0]
        A = np.zeros((5, 5))
        A[0, 0] = 1.0
        A[0, 1] = time_s
        A[1, 2] = curr_spd
        A[2, 2] = 1.0
        A[2, 3] = time_s
        A[4, 4] = 1.0

        # B = [0.0, 0.0
        #     0.0, 0.0
        #     0.0, 0.0
        #     v/L, 0.0
        #     0.0, dt]
        B = np.zeros((5, 2))
        B[3, 0] = curr_spd / self.WHEEL_BASE_M
        B[4, 1] = time_s

        gain = self._calculate_control_gain(A, B)

        # state vector
        x = np.zeros((5, 1))
        x[0, 0] = error_lat_m # lateral error against course
        x[1, 0] = (error_lat_m - self.prev_error_lat_m) / time_s # derivative of lateral error
        x[2, 0] = error_yaw_rad # yaw angle error against course
        x[3, 0] = (error_yaw_rad - self.prev_error_yaw_rad) / time_s # derivative of yaw angle error
        x[4, 0] = curr_spd - self.target_speed_mps # speed error against target value
        self.prev_error_lat_m = error_lat_m
        self.prev_error_yaw_rad = error_yaw_rad

        # feedback input vector
        # [[steering angle],
        #  [acceleration]]
        feedback_input = -gain @ x

        # target steering angle
        feedforward_steer = atan2(self.WHEEL_BASE_M * trgt_curv, 1)
        feedback_steer = pi_to_pi(feedback_input[0, 0])
        self.target_steer_rad = feedforward_steer + feedback_steer

        # target yaw rate
        self.target_yaw_rate_rps = curr_spd * tan(self.target_steer_rad) / self.WHEEL_BASE_M

        # target acceleration
        self.target_accel_mps2 = feedback_input[1, 0]

    def update(self, state, time_s):
        """
        Function to update data for path tracking
        state: Vehicle's state object
        time_s: Simulation interval time[sec]
        """

        if not self.course: return

        self._calculate_target_course_index(state)

        self._decide_target_speed_mps()

        _, error_lat_m, error_yaw_rad = self._calculate_tracking_error(state)

        self._calculate_control_input(state, error_lat_m, error_yaw_rad, time_s)

    def get_target_accel_mps2(self):
        """
        Function to get acceleration input[m/s2]
        """
        
        return self.target_accel_mps2

    def get_target_yaw_rate_rps(self):
        """
        Function to get yaw rate input[rad/s]
        """

        return self.target_yaw_rate_rps

    def get_target_steer_rad(self):
        """
        Function to get steering angle input[rad]
        """
        
        return self.target_steer_rad

    def draw(self, axes, elems):
        """
        Function to draw target point on course
        axes: Axes object of figure
        elems: plot object's list
        """

        pass

"""
pid_controller.py

Author: Shisato Yano
"""

from pathlib import Path
from math import sin, tan, atan2, atan, pi
import sys
import numpy as np

sys.path.append(str(Path(__file__).absolute().parent) + "/../../common")
from matrix_lib import rot_mat_22


class PidController:
    """
    Controller class by PID control algorithm
    """

    def __init__(self, spec, course=None):
        """
        Constructor
        spec: Vehicle specification object
        course: Course data and logic object
        """

        self.MIN_LOOK_AHEAD_DISTANCE_M = 2.0
        self.SPEED_PROPORTIONAL_GAIN = 1.0
        self.WHEEL_BASE_M = spec.wheel_base_m

        self.course = course
        self.look_ahead_distance_m = self.MIN_LOOK_AHEAD_DISTANCE_M
        self.target_course_index = 0
        self.feedforward_steer_rad = 0.0
        self.target_accel_mps2 = 0.0
        self.target_steer_rad = 0.0
        self.target_yaw_rate_rps = 0.0

    def _calculate_target_course_index(self, state):
        """
        Private function to calculate target point's index on course
        state: Vehicle's state object
        """
        
        self.target_course_index = self.course.search_nearest_point_index(state)

    def _calculate_target_acceleration_mps2(self, state):
        """
        Private function to calculate acceleration input
        state: Vehicle's state object
        """

        diff_speed_mps = self.course.calculate_speed_difference_mps(state, self.target_course_index)
        self.target_accel_mps2 = self.SPEED_PROPORTIONAL_GAIN * diff_speed_mps

    def _calculate_feedforward_steer_angle_rad(self, state):
        """
        Private function to calculate steering angle input
        state: Vehicle's state object
        """
        
        target_curvature = self.course.target_point_curvature(self.target_course_index)
        self.feedforward_steer_rad = atan(self.WHEEL_BASE_M * target_curvature)

    def _calculate_target_steer_rad(self, state):
        target_x_m = self.course.point_x_m(self.target_course_index)
        target_y_m = self.course.point_y_m(self.target_course_index)
        current_x_m = state.get_x_m()
        current_y_m = state.get_y_m()

        error_x_m = current_x_m - target_x_m
        error_y_m = current_y_m - target_y_m
        error_xy_m = np.array([[error_x_m],
                               [error_y_m]])
        current_yaw_rad = state.get_yaw_rad()
        rot_mat = rot_mat_22(-current_yaw_rad)
        error_lonlat_m = rot_mat @ error_xy_m
        error_yaw_rad = current_yaw_rad - atan2(error_y_m, error_x_m)

        while (not(-2*pi <= error_yaw_rad <= 2*pi)):
            if (error_yaw_rad >= 2*pi): error_yaw_rad -= 2*pi
            elif (error_yaw_rad <= -2*pi): error_yaw_rad += 2*pi
        if error_yaw_rad > pi: error_yaw_rad -= 2*pi
        elif error_yaw_rad < -pi: error_yaw_rad += 2*pi

        self.target_steer_rad = -0.2 * error_lonlat_m[1, 0] - 0.4 * error_yaw_rad + self.feedforward_steer_rad
        print(error_lonlat_m[1, 0])


    def _calculate_target_yaw_rate_rps(self, state):
        """
        Private function to calculate yaw rate input
        state: Vehicle's state object
        """

        self.target_yaw_rate_rps = state.get_speed_mps() * tan(self.target_steer_rad) / self.WHEEL_BASE_M

    def update(self, state):
        """
        Function to update data for path tracking
        state: Vehicle's state object
        """

        if not self.course: return

        self._calculate_target_course_index(state)

        self._calculate_target_acceleration_mps2(state)

        self._calculate_feedforward_steer_angle_rad(state)

        self._calculate_target_steer_rad(state)

        self._calculate_target_yaw_rate_rps(state)
    
    def get_target_accel_mps2(self):
        """
        Function to get acceleration input[m/s2]
        """
        
        return self.target_accel_mps2
    
    def get_target_steer_rad(self):
        """
        Function to get steering angle input[rad]
        """
        
        return self.target_steer_rad

    def get_target_yaw_rate_rps(self):
        """
        Function to get yaw rate input[rad/s]
        """

        return self.target_yaw_rate_rps
    
    def draw(self, axes, elems):
        pass

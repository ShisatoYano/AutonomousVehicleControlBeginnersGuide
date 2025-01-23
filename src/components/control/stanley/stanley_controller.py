"""
stanley_controller.py

Author: Shisato Yano
"""

# import path setting
import sys
from pathlib import Path
from math import sin, cos, tan, atan2

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"

sys.path.append(abs_dir_path + relative_path + "state")

#import component modules
from state import State


class StanleyController:
    """
    Controller class by Stanley steering control algorithm
    """

    def __init__(self, spec, course=None):
        """
        Constructor
        course: Course data and logic object
        """

        self.SPEED_PROPORTIONAL_GAIN = 1.0
        self.CONTROL_GAIN = 0.1
        self.WHEEL_BASE_M = spec.wheel_base_m

        self.course = course
        self.target_course_index = 0
        self.target_accel_mps2 = 0.0
        self.target_speed_mps = 0.0
        self.target_yaw_rate_rps = 0.0
        self.target_steer_rad = 0.0
    
    def _calculate_front_axle_state(self, state):
        """
        Private function to calculate state of front axle
        state: State object of vehicle's rear axle
        Return state object of vehicle's front axle
        """

        # current rear axle position and pose
        curr_rear_x = state.get_x_m()
        curr_rear_y = state.get_y_m()
        curr_yaw = state.get_yaw_rad()
        curr_spd = state.get_speed_mps()

        # calculate front axle position
        curr_front_x = curr_rear_x + self.WHEEL_BASE_M * cos(curr_yaw)
        curr_front_y = curr_rear_y + self.WHEEL_BASE_M * sin(curr_yaw)
        curr_front_state = State(curr_front_x, curr_front_y, curr_yaw, curr_spd)

        return curr_front_state

    def _calculate_target_course_index(self, state):
        """
        Private function to calculate target point's index on course
        state: State object of vehicle's front axle
        """
        
        nearest_index = self.course.search_nearest_point_index(state)
        self.target_course_index = nearest_index

    def _decide_target_speed_mps(self):
        """
        Private function to decide target speed[m/s]
        """

        self.target_speed_mps = self.course.point_speed_mps(self.target_course_index)

    def _calculate_target_acceleration_mps2(self, state):
        """
        Private function to calculate acceleration input
        state: State object of vehicle's front axle
        """

        diff_speed_mps = self.target_speed_mps - state.get_speed_mps()
        self.target_accel_mps2 = self.SPEED_PROPORTIONAL_GAIN * diff_speed_mps

    def _calculate_tracking_error(self, state):
        """
        Private function to calculate tracking error against target point on the course
        state: State object of vehicle's front axle
        """

        error_lon_m, error_lat_m, error_yaw_rad = self.course.calculate_lonlat_error(state, self.target_course_index)
        return error_lon_m, error_lat_m, error_yaw_rad

    def _calculate_control_input(self, state, error_lat_m, error_yaw_rad):
        """
        Private function to calculate yaw rate input
        state: State object of vehicle's front axle
        error_lat_m: Lateral error against reference course[m]
        error_yaw_rad: Yaw angle error against reference course[rad]
        """

        # calculate steering angle input
        curr_spd = state.get_speed_mps()
        error_steer_rad = atan2(self.CONTROL_GAIN * error_lat_m, curr_spd)
        self.target_steer_rad = error_steer_rad + error_yaw_rad

        # calculate yaw rate input
        self.target_yaw_rate_rps = curr_spd * tan(self.target_steer_rad) / self.WHEEL_BASE_M

    def update(self, state, time_s):
        """
        Function to update data for path tracking
        state: Vehicle's state object
        time_s: Simulation interval time[sec]
        """

        if not self.course: return

        front_axle_state = self._calculate_front_axle_state(state)

        self._calculate_target_course_index(front_axle_state)

        self._decide_target_speed_mps()

        self._calculate_target_acceleration_mps2(front_axle_state)

        _, error_lat_m, error_yaw_rad = self._calculate_tracking_error(front_axle_state)
        print(error_lat_m, error_yaw_rad)

        self._calculate_control_input(front_axle_state, error_lat_m, error_yaw_rad)

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

"""
rear_wheel_feedback_controller.py

Author: Shisato Yano
"""

from math import sin, cos, atan2


class RearWheelFeedbackController:
    """
    Controller class by Rear wheel feedback algorithm
    """

    def __init__(self, spec, course=None):
        """
        Constructor
        course: Course data and logic object
        """

        self.SPEED_PROPORTIONAL_GAIN = 1.0
        self.YAW_ERROR_GAIN = 1.0
        self.LAT_ERROR_GAIN = 0.5
        self.WHEEL_BASE_M = spec.wheel_base_m
        self.MAX_ACCEL_MPS2 = spec.max_accel_mps2

        self.course = course
        self.target_course_index = 0
        self.target_accel_mps2 = 0.0
        self.target_speed_mps = 0.0
        self.target_yaw_rate_rps = 0.0
        self.target_steer_rad = 0.0
        self.elapsed_time_sec = 0.0

        self.max_spd_mps = self.course.max_speed_mps()
        self.accel_time_s = self.max_spd_mps / self.MAX_ACCEL_MPS2
        self.decel_time_s = self.accel_time_s
        accel_dist_m = self.max_spd_mps * self.accel_time_s / 2
        decel_dist_m = self.max_spd_mps * self.decel_time_s / 2
        self.const_time_s = (self.course.distance_m() - accel_dist_m - decel_dist_m) / self.max_spd_mps
    
    def _calculate_target_course_index(self, state):
        """
        Private function to calculate target point's index on course
        state: Vehicle's state object
        """
        
        nearest_index = self.course.search_nearest_point_index(state)
        self.target_course_index = nearest_index
    
    def _decide_target_speed_mps(self, time_s):
        """
        Private function to decide target speed[m/s]
        time_s: interval time[sec]
        """
        
        if self.elapsed_time_sec <= self.accel_time_s:
            self.target_speed_mps += self.MAX_ACCEL_MPS2 * time_s
            if self.target_speed_mps >= self.max_spd_mps:
                self.target_speed_mps = self.max_spd_mps
        elif self.accel_time_s < self.elapsed_time_sec <= (self.accel_time_s + self.const_time_s):
            self.target_speed_mps = self.max_spd_mps
        else:
            self.target_speed_mps -= self.MAX_ACCEL_MPS2 * time_s
            if self.target_speed_mps <= 0.0:
                self.target_speed_mps = 0.0

    def _calculate_target_acceleration_mps2(self, state):
        """
        Private function to calculate acceleration input
        state: Vehicle's state object
        """

        diff_speed_mps = self.target_speed_mps - state.get_speed_mps()
        self.target_accel_mps2 = self.SPEED_PROPORTIONAL_GAIN * diff_speed_mps

    def _calculate_tracking_error(self, state):
        """
        Private function to calculate tracking error against target point on the course
        state: Vehicle's state object
        """

        error_lon_m, error_lat_m, error_yaw_rad = self.course.calculate_lonlat_error(state, self.target_course_index)
        return error_lon_m, error_lat_m, error_yaw_rad
    
    def _calculate_target_yaw_rate_rps(self, state, error_lat_m, error_yaw_rad):
        """
        Private function to calculate yaw rate input
        state: Vehicle's state object
        error_lat_m: Lateral error against reference course[m]
        error_yaw_rad: Yaw angle error against reference course[rad]
        """

        curr_spd = state.get_speed_mps()
        trgt_curv = self.course.point_curvature(self.target_course_index)

        if error_yaw_rad == 0.0:
            yaw_rate_rps = 0.0
        else:
            yaw_rate_rps = curr_spd * trgt_curv * cos(error_yaw_rad) / (1.0 - trgt_curv * error_lat_m) - \
                self.YAW_ERROR_GAIN * abs(curr_spd) * error_yaw_rad - \
                    self.LAT_ERROR_GAIN * curr_spd * sin(error_yaw_rad) * error_lat_m / error_yaw_rad
        self.target_yaw_rate_rps = yaw_rate_rps

    def _calculate_target_steer_angle_rad(self, state):
        """
        Private function to calculate steering angle input
        state: Vehicle's state object
        """

        if abs(state.get_speed_mps()) != 0.0:
            self.target_steer_rad = atan2(self.WHEEL_BASE_M * self.target_yaw_rate_rps / state.get_speed_mps(), 1.0)
        else:
            self.target_steer_rad = 0.0

    def update(self, state, time_s):
        """
        Function to update data for path tracking
        state: Vehicle's state object
        time_s: Simulation interval time[sec]
        """

        if not self.course: return

        self._calculate_target_course_index(state)

        self._decide_target_speed_mps(time_s)

        self._calculate_target_acceleration_mps2(state)

        _, error_lat_m, error_yaw_rad = self._calculate_tracking_error(state)

        self._calculate_target_yaw_rate_rps(state, error_lat_m, error_yaw_rad)

        self._calculate_target_steer_angle_rad(state)

        self.elapsed_time_sec += time_s
    
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

"""
pid_controller.py

Author: Shisato Yano
"""

from math import sin, tan, atan2


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
        self.LOOK_FORWARD_GAIN = 0.3
        self.SPEED_PROPORTIONAL_GAIN = 1.0
        self.WHEEL_BASE_M = spec.wheel_base_m

        self.course = course
        self.look_ahead_distance_m = self.MIN_LOOK_AHEAD_DISTANCE_M
        self.target_course_index = 0
        self.target_accel_mps2 = 0.0
        self.target_steer_rad = 0.0
        self.target_yaw_rate_rps = 0.0
    
    def _calculate_look_ahead_distance(self, state):
        """
        Private function to calculate look ahead distance to target point
        state: Vehicle's state object
        """
        
        self.look_ahead_distance_m = self.LOOK_FORWARD_GAIN * state.get_speed_mps() + self.MIN_LOOK_AHEAD_DISTANCE_M

    def _calculate_target_course_index(self, state):
        """
        Private function to calculate target point's index on course
        state: Vehicle's state object
        """
        
        nearest_index = self.course.search_nearest_point_index(state)
        while self.look_ahead_distance_m > self.course.calculate_distance_from_point(state, nearest_index):
            if nearest_index + 1 >= self.course.length(): break
            nearest_index += 1
        self.target_course_index = nearest_index

    def update(self, state):
        """
        Function to update data for path tracking
        state: Vehicle's state object
        """

        if not self.course: return

        self._calculate_look_ahead_distance(state)

        self._calculate_target_course_index(state)

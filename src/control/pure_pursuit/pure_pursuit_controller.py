"""
pure_pursuit_controller.py

Author: Shisato Yano
"""

from math import sin, tan, atan2


class PurePursuitController:
    def __init__(self, spec, course=None):
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
    
    def calculate_look_ahead_distance(self, state):
        self.look_ahead_distance_m = self.LOOK_FORWARD_GAIN * state.get_speed_mps() + self.MIN_LOOK_AHEAD_DISTANCE_M

    def calculate_target_course_index(self, state):
        nearest_index = self.course.search_nearest_point_index(state)
        while self.look_ahead_distance_m > self.course.calculate_distance_from_point(state, nearest_index):
            if nearest_index + 1 >= self.course.length(): break
            nearest_index += 1
        self.target_course_index = nearest_index

    def calculate_target_acceleration_mps2(self, state):
        diff_speed_mps = self.course.calculate_speed_difference_mps(state, self.target_course_index)
        self.target_accel_mps2 = self.SPEED_PROPORTIONAL_GAIN * diff_speed_mps

    def calculate_target_steer_angle_rad(self, state):
        diff_angle_rad = self.course.calculate_angle_difference_rad(state, self.target_course_index)
        self.target_steer_rad = atan2((2 * self.WHEEL_BASE_M * sin(diff_angle_rad)), self.look_ahead_distance_m)

    def calculate_target_yaw_rate_rps(self, state):
        self.target_yaw_rate_rps = state.get_speed_mps() * tan(self.target_steer_rad) / self.WHEEL_BASE_M

    def update(self, state):
        if not self.course: return

        self.calculate_look_ahead_distance(state)
        
        self.calculate_target_course_index(state)

        self.calculate_target_acceleration_mps2(state)

        self.calculate_target_steer_angle_rad(state)

        self.calculate_target_yaw_rate_rps(state)
    
    def get_target_accel_mps2(self):
        return self.target_accel_mps2
    
    def get_target_steer_rad(self):
        return self.target_steer_rad

    def get_target_yaw_rate_rps(self):
        return self.target_yaw_rate_rps
    
    def draw(self, axes, elems):
        elems += axes.plot(self.course.point_x_m(self.target_course_index), 
                           self.course.point_y_m(self.target_course_index), 
                           marker='o', 
                           color='g',
                           linewidth=0, 
                           label="Target Point")

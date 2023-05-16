"""
pure_pursuit_controller.py

Author: Shisato Yano
"""

from math import sin, tan, atan2


class PurePursuitController:
    """
    Controller class by Pure Pursuit algorithm
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
    
    def calculate_look_ahead_distance(self, state):
        """
        Function to calculate look ahead distance to target point
        state: Vehicle's state object
        """
        
        self.look_ahead_distance_m = self.LOOK_FORWARD_GAIN * state.get_speed_mps() + self.MIN_LOOK_AHEAD_DISTANCE_M

    def calculate_target_course_index(self, state):
        """
        Function to calculate target point's index on course
        state: Vehicle's state object
        """
        
        nearest_index = self.course.search_nearest_point_index(state)
        while self.look_ahead_distance_m > self.course.calculate_distance_from_point(state, nearest_index):
            if nearest_index + 1 >= self.course.length(): break
            nearest_index += 1
        self.target_course_index = nearest_index

    def calculate_target_acceleration_mps2(self, state):
        """
        Function to calculate acceleration input
        state: Vehicle's state object
        """

        diff_speed_mps = self.course.calculate_speed_difference_mps(state, self.target_course_index)
        self.target_accel_mps2 = self.SPEED_PROPORTIONAL_GAIN * diff_speed_mps

    def calculate_target_steer_angle_rad(self, state):
        """
        Function to calculate steering angle input
        state: Vehicle's state object
        """
        
        diff_angle_rad = self.course.calculate_angle_difference_rad(state, self.target_course_index)
        self.target_steer_rad = atan2((2 * self.WHEEL_BASE_M * sin(diff_angle_rad)), self.look_ahead_distance_m)

    def calculate_target_yaw_rate_rps(self, state):
        """
        Function to calculate yaw rate input
        state: Vehicle's state object
        """

        self.target_yaw_rate_rps = state.get_speed_mps() * tan(self.target_steer_rad) / self.WHEEL_BASE_M

    def update(self, state):
        """
        Function to update data for path tracking
        state: Vehicle's state object
        """
        
        if not self.course: return

        self.calculate_look_ahead_distance(state)
        
        self.calculate_target_course_index(state)

        self.calculate_target_acceleration_mps2(state)

        self.calculate_target_steer_angle_rad(state)

        self.calculate_target_yaw_rate_rps(state)
    
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
        """
        Function to draw target point on course
        axes: Axes object of figure
        elems: plot object's list
        """

        target_point_plot, = axes.plot(self.course.point_x_m(self.target_course_index), 
                                       self.course.point_y_m(self.target_course_index), 
                                       marker='o', 
                                       color='g',
                                       linewidth=0, 
                                       label="Target Point")
        elems.append(target_point_plot)

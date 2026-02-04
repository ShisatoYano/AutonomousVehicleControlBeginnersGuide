"""
adaptive_pure_pursuit_controller.py

Adaptive Pure Pursuit path tracking controller. Extends pure pursuit by adapting
the look-ahead distance using speed and path curvature: shorter look-ahead on
sharp curves for better tracking, longer look-ahead on straights for stability.
"""

from math import sin, tan, atan2


class AdaptivePurePursuitController:
    """
    Controller class by Adaptive Pure Pursuit algorithm.
    Look-ahead distance is adapted by speed (longer at higher speed) and by
    path curvature at the current nearest point (shorter on sharp curves).
    """

    def __init__(self, spec, course=None, color='g',
                 min_look_ahead_m=2.0,
                 look_forward_gain=0.3,
                 curvature_adapt_gain=1.0,
                 max_look_ahead_m=15.0,
                 speed_proportional_gain=1.0):
        """
        Constructor
        spec: Vehicle specification object
        course: Course data and logic object (with point_curvature optional)
        color: Color of drawing target point
        min_look_ahead_m: Minimum look-ahead distance [m]
        look_forward_gain: Gain for speed-proportional look-ahead
        curvature_adapt_gain: Gain for curvature-based reduction (0 = disable)
        max_look_ahead_m: Maximum look-ahead distance [m]
        speed_proportional_gain: Gain for speed tracking
        """
        self.MIN_LOOK_AHEAD_DISTANCE_M = min_look_ahead_m
        self.LOOK_FORWARD_GAIN = look_forward_gain
        self.CURVATURE_ADAPT_GAIN = curvature_adapt_gain
        self.MAX_LOOK_AHEAD_DISTANCE_M = max_look_ahead_m
        self.SPEED_PROPORTIONAL_GAIN = speed_proportional_gain
        self.WHEEL_BASE_M = spec.wheel_base_m
        self.DRAW_COLOR = color

        self.course = course
        self.look_ahead_distance_m = self.MIN_LOOK_AHEAD_DISTANCE_M
        self.target_course_index = 0
        self.target_accel_mps2 = 0.0
        self.target_speed_mps = 0.0
        self.target_steer_rad = 0.0
        self.target_yaw_rate_rps = 0.0

    def _calculate_look_ahead_distance(self, state):
        """
        Adaptive look-ahead: base term from speed, reduced by path curvature.
        L = clamp((L0 + k * v) / (1 + K_curv * |curvature|), L_min, L_max)
        """
        base_look_ahead = self.LOOK_FORWARD_GAIN * state.get_speed_mps() + self.MIN_LOOK_AHEAD_DISTANCE_M

        curvature_factor = 1.0
        if self.course and hasattr(self.course, 'point_curvature'):
            nearest_index = self.course.search_nearest_point_index(state)
            curvature = self.course.point_curvature(nearest_index)
            curvature_factor = 1.0 + self.CURVATURE_ADAPT_GAIN * abs(curvature)

        self.look_ahead_distance_m = base_look_ahead / curvature_factor
        self.look_ahead_distance_m = max(
            self.MIN_LOOK_AHEAD_DISTANCE_M,
            min(self.MAX_LOOK_AHEAD_DISTANCE_M, self.look_ahead_distance_m),
        )

    def _calculate_target_course_index(self, state):
        """
        Private function to calculate target point's index on course
        state: Vehicle's state object
        """
        nearest_index = self.course.search_nearest_point_index(state)
        while self.look_ahead_distance_m > self.course.calculate_distance_from_point(state, nearest_index):
            if nearest_index + 1 >= self.course.length():
                break
            nearest_index += 1
        self.target_course_index = nearest_index

    def _decide_target_speed_mps(self):
        """Private function to decide target speed [m/s]."""
        self.target_speed_mps = self.course.point_speed_mps(self.target_course_index)

    def _calculate_target_acceleration_mps2(self, state):
        """Private function to calculate acceleration input."""
        diff_speed_mps = self.target_speed_mps - state.get_speed_mps()
        self.target_accel_mps2 = self.SPEED_PROPORTIONAL_GAIN * diff_speed_mps

    def _calculate_target_steer_angle_rad(self, state):
        """Private function to calculate steering angle input."""
        diff_angle_rad = self.course.calculate_angle_difference_rad(state, self.target_course_index)
        self.target_steer_rad = atan2(
            (2 * self.WHEEL_BASE_M * sin(diff_angle_rad)),
            self.look_ahead_distance_m,
        )

    def _calculate_target_yaw_rate_rps(self, state):
        """Private function to calculate yaw rate input."""
        self.target_yaw_rate_rps = state.get_speed_mps() * tan(self.target_steer_rad) / self.WHEEL_BASE_M

    def update(self, state, time_s):
        """
        Function to update data for path tracking
        state: Vehicle's state object
        time_s: Simulation interval time [sec]
        """
        if not self.course:
            return

        self._calculate_look_ahead_distance(state)
        self._calculate_target_course_index(state)
        self._decide_target_speed_mps()
        self._calculate_target_acceleration_mps2(state)
        self._calculate_target_steer_angle_rad(state)
        self._calculate_target_yaw_rate_rps(state)

    def get_target_accel_mps2(self):
        """Function to get acceleration input [m/s^2]."""
        return self.target_accel_mps2

    def get_target_steer_rad(self):
        """Function to get steering angle input [rad]."""
        return self.target_steer_rad

    def get_target_yaw_rate_rps(self):
        """Function to get yaw rate input [rad/s]."""
        return self.target_yaw_rate_rps

    def draw(self, axes, elems):
        """Function to draw target point on course."""
        target_point_plot, = axes.plot(
            self.course.point_x_m(self.target_course_index),
            self.course.point_y_m(self.target_course_index),
            marker='o',
            color=self.DRAW_COLOR,
            linewidth=0,
            label="Target Point",
        )
        elems.append(target_point_plot)

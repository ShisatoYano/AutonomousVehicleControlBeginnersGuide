"""
adaptive_pure_pursuit_controller.py

Author: Erwin Lejeune

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

    Like Pure Pursuit, it takes the course point at the look-ahead distance
    l_d ahead of the rear axle, and steers along the circular arc that starts
    at the rear axle in the heading direction and passes through that point.
    That arc is the ideal geometry: the code takes the first course point
    at least l_d away and still uses l_d in (4), as the Algorithm part of
    4.2 explains.
    The equation numbers below are those of section 4.2 of the book:
    https://shisatoyano.github.io/AutonomousVehicleControlBeginnersGuide/path_tracking/pure_pursuit.html

    Reference
        R. C. Coulter, "Implementation of the Pure Pursuit Path Tracking
        Algorithm", Technical Report CMU-RI-TR-92-01, Carnegie Mellon
        University, 1992.
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
        l_d = min(max((k * v + l_d_min) / (1 + k_kappa * |kappa_n|), l_d_min), l_d_max)    (8)
        k: LOOK_FORWARD_GAIN[s], k_kappa: CURVATURE_ADAPT_GAIN[m],
        kappa_n: curvature of the course point nearest to the rear axle[1/m],
        l_d_min, l_d_max: MIN_LOOK_AHEAD_DISTANCE_M, MAX_LOOK_AHEAD_DISTANCE_M
        The code clamps as max(l_d_min, min(l_d_max, ...)), the same as (8)
        while l_d_min <= l_d_max. A course without point_curvature gives
        kappa_n = 0 in (8).
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
        From the course point nearest to the rear axle, walk forward to the
        first point at least l_d away: the look-ahead point (step 2 of the
        algorithm in 4.2). The nearest point is used if it is already that
        far, and the last course point if no point is.
        state: Vehicle's state object
        """
        nearest_index = self.course.search_nearest_point_index(state)
        while self.look_ahead_distance_m > self.course.calculate_distance_from_point(state, nearest_index):
            if nearest_index + 1 >= self.course.length():
                break
            nearest_index += 1
        self.target_course_index = nearest_index

    def _decide_target_speed_mps(self):
        """
        Private function to decide target speed [m/s].
        v_r: the course's target speed at the look-ahead point
        """
        self.target_speed_mps = self.course.point_speed_mps(self.target_course_index)

    def _calculate_target_acceleration_mps2(self, state):
        """
        Private function to calculate acceleration input.
        a = k_v * (v_r - v)    ("Speed" in 4.2, not numbered)
        k_v: SPEED_PROPORTIONAL_GAIN
        """
        diff_speed_mps = self.target_speed_mps - state.get_speed_mps()
        self.target_accel_mps2 = self.SPEED_PROPORTIONAL_GAIN * diff_speed_mps

    def _calculate_target_steer_angle_rad(self, state):
        """
        Private function to calculate steering angle input.
        delta = atan(2 * L * sin(alpha) / l_d)    (4)
        alpha = atan2(y_p - y, x_p - x) - theta    (1)
        alpha is diff_angle_rad, the angle from the heading to the look-ahead
        point (x_p, y_p). L: WHEEL_BASE_M. The code writes (4) as
        atan2(2 * L * sin(alpha), l_d), which is the same because l_d > 0.
        """
        diff_angle_rad = self.course.calculate_angle_difference_rad(state, self.target_course_index)
        self.target_steer_rad = atan2(
            (2 * self.WHEEL_BASE_M * sin(diff_angle_rad)),
            self.look_ahead_distance_m,
        )

    def _calculate_target_yaw_rate_rps(self, state):
        """
        Private function to calculate yaw rate input.
        omega = v * tan(delta) / L    (5)
        """
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

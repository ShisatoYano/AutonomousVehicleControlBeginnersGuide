"""
vfh_controller.py

Author: Khushi
"""

import sys
from pathlib import Path
from math import atan2, pi

sys.path.append(str(Path(__file__).absolute().parent) + "/../../common")
from angle_lib import pi_to_pi


class VfhController:
    """
    Controller class to drive a vehicle by Vector Field Histogram(VFH)
    based reactive obstacle avoidance. Each cycle, it reads the global
    frame direction currently selected by a mapper's direction selector
    (see DirectionSelector, Step 3 of the VFH roadmap) and converts it
    into acceleration / yaw rate inputs through simple proportional
    control. Target speed is the configured cruise speed, scaled down
    toward min_speed_mps as the smoothed obstacle density ahead - read
    from the mapper's polar histogram, in the current target direction -
    rises toward danger_density(Step 5 of the VFH roadmap: Dynamic Speed
    Control)
    """

    def __init__(self, spec, mapper, cruise_speed_mps=3.0,
                 speed_gain=1.0, yaw_rate_gain=1.5,
                 max_yaw_rate_rps=1.0, min_speed_mps=0.0,
                 danger_density=1.0, caution_half_angle_rad=pi / 18,
                 color='b'):
        """
        Constructor
        spec: Vehicle specification object
        mapper: Mapper object exposing get_direction_selector(), whose
                DirectionSelector exposes get_selected_angle_rad(), and
                get_histogram(), whose PolarHistogram exposes
                max_density_in_angle_range()
        cruise_speed_mps: Target speed[m/s] with no obstacle ahead
        speed_gain: Proportional gain from speed error to acceleration
        yaw_rate_gain: Proportional gain from heading error to yaw rate
        max_yaw_rate_rps: Saturation limit of commanded yaw rate[rad/s]
        min_speed_mps: Target speed[m/s] once density ahead reaches
                       danger_density or higher(Step 5: Dynamic Speed Control)
        danger_density: Smoothed density ahead at/above which target speed
                        is already down to min_speed_mps. Below it, target
                        speed scales linearly between cruise_speed_mps
                        (density 0) and min_speed_mps(density danger_density)
        caution_half_angle_rad: Half width[rad], to each side of the target
                                 direction, of the angular range checked for
                                 obstacle density. Looking slightly wider
                                 than a single sector avoids being fooled by
                                 noise right at a sector boundary
        color: Reserved for drawing, kept for interface parity with
               other controllers
        """

        if cruise_speed_mps < 0.0:
            raise ValueError("cruise speed must be 0 or greater")
        if speed_gain < 0.0 or yaw_rate_gain < 0.0:
            raise ValueError("gains must be 0 or greater")
        if max_yaw_rate_rps <= 0.0:
            raise ValueError("max yaw rate must be greater than 0")
        if min_speed_mps < 0.0 or min_speed_mps > cruise_speed_mps:
            raise ValueError("min speed must be between 0 and cruise speed")
        if danger_density <= 0.0:
            raise ValueError("danger density must be greater than 0")
        if caution_half_angle_rad < 0.0:
            raise ValueError("caution half angle must be 0 or greater")

        self.WHEEL_BASE_M = spec.wheel_base_m
        self.DRAW_COLOR = color

        self.mapper = mapper
        self.cruise_speed_mps = float(cruise_speed_mps)
        self.speed_gain = float(speed_gain)
        self.yaw_rate_gain = float(yaw_rate_gain)
        self.max_yaw_rate_rps = float(max_yaw_rate_rps)
        self.min_speed_mps = float(min_speed_mps)
        self.danger_density = float(danger_density)
        self.caution_half_angle_rad = float(caution_half_angle_rad)

        self.target_angle_rad = None
        self.target_speed_mps = self.cruise_speed_mps
        self.target_accel_mps2 = 0.0
        self.target_yaw_rate_rps = 0.0
        self.target_steer_rad = 0.0

    def _decide_target_direction_rad(self, state):
        """
        Private function to decide the global frame angle to steer
        toward this cycle. Falls back to the vehicle's current heading
        when the mapper has no direction selected yet, for example
        when every sector is blocked and no valley exists
        state: Vehicle's state object
        """

        selected_angle_rad = self.mapper.get_direction_selector().get_selected_angle_rad()
        if selected_angle_rad is None:
            self.target_angle_rad = state.get_yaw_rad()
        else:
            self.target_angle_rad = selected_angle_rad

    def _decide_target_speed_mps(self, state):
        """
        Private function to decide the target speed this cycle: the
        configured cruise speed, scaled down toward min_speed_mps as the
        obstacle density ahead - in the current target direction, read
        from the mapper's polar histogram - rises toward danger_density
        (Step 5 of the VFH roadmap: Dynamic Speed Control)
        state: Vehicle's state object
        """

        heading_relative_angle_rad = pi_to_pi(self.target_angle_rad - state.get_yaw_rad())
        density_ahead = self.mapper.get_histogram().max_density_in_angle_range(
            heading_relative_angle_rad, self.caution_half_angle_rad)

        speed_scale = 1.0 - density_ahead / self.danger_density
        if speed_scale < 0.0:
            speed_scale = 0.0
        elif speed_scale > 1.0:
            speed_scale = 1.0

        self.target_speed_mps = self.min_speed_mps + speed_scale * (self.cruise_speed_mps - self.min_speed_mps)

    def _calculate_target_acceleration_mps2(self, state):
        """
        Private function to calculate acceleration input by simple
        proportional control toward this cycle's target speed(see
        _decide_target_speed_mps)
        state: Vehicle's state object
        """

        diff_speed_mps = self.target_speed_mps - state.get_speed_mps()
        self.target_accel_mps2 = self.speed_gain * diff_speed_mps

    def _calculate_target_yaw_rate_rps(self, state):
        """
        Private function to calculate yaw rate input by simple
        proportional control toward the target direction, saturated to
        the configured maximum magnitude
        state: Vehicle's state object
        """

        diff_angle_rad = pi_to_pi(self.target_angle_rad - state.get_yaw_rad())
        yaw_rate_rps = self.yaw_rate_gain * diff_angle_rad

        if yaw_rate_rps > self.max_yaw_rate_rps:
            yaw_rate_rps = self.max_yaw_rate_rps
        elif yaw_rate_rps < -self.max_yaw_rate_rps:
            yaw_rate_rps = -self.max_yaw_rate_rps

        self.target_yaw_rate_rps = yaw_rate_rps

    def _calculate_target_steer_rad(self, state):
        """
        Private function to back out a front tire steering angle from
        the commanded yaw rate, for visualization only, using the
        bicycle model relation yaw_rate = speed * tan(steer) / wheel_base
        state: Vehicle's state object
        """

        speed_mps = state.get_speed_mps()
        if abs(speed_mps) < 1e-3:
            self.target_steer_rad = 0.0
        else:
            self.target_steer_rad = atan2(self.WHEEL_BASE_M * self.target_yaw_rate_rps, speed_mps)

    def update(self, state, time_s):
        """
        Function to update data for VFH based obstacle avoidance driving
        state: Vehicle's state object
        time_s: Simulation interval time[sec]
        """

        self._decide_target_direction_rad(state)

        self._decide_target_speed_mps(state)

        self._calculate_target_acceleration_mps2(state)

        self._calculate_target_yaw_rate_rps(state)

        self._calculate_target_steer_rad(state)

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
        Function to get steering angle input[rad], for visualization only
        """

        return self.target_steer_rad

    def get_target_angle_rad(self):
        """
        Function to get the global frame angle currently targeted
        """

        return self.target_angle_rad

    def get_target_speed_mps(self):
        """
        Function to get this cycle's target speed[m/s]: the cruise speed
        scaled down by nearby obstacle density(Step 5: Dynamic Speed Control)
        """

        return self.target_speed_mps

    def draw(self, axes, elems):
        """
        Function to draw controller data. The targeted direction is
        already drawn by the mapper's own selection arrow(Step 3), so
        this intentionally adds nothing to avoid a duplicate overlay
        axes: Axes object of figure
        elems: List of plot object
        """

        pass

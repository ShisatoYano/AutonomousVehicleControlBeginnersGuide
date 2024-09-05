"""
cubic_spline_2d.py

Author: Shisato Yano
"""

import numpy as np
import math

from cubic_spline import CubicSpline


class CubicSpline2D:
    """
    2D Cubic Spline class
    Each section between two points are approximated as the following equation
    y_i = a_i + b_i * (x - x_i) + c_i * (x - x_i)^2 + d_i * (x - x_i)^3
    """

    def __init__(self, x_points, y_points):
        self.s = self._calc_base_points(x_points, y_points)
        self.sx = CubicSpline(self.s, x_points)
        self.sy = CubicSpline(self.s, y_points)
    
    def calc_interpolated_xy(self, s):
        interpolated_x = self.sx.calculate_position(s)
        interpolated_y = self.sy.calculate_position(s)

        return interpolated_x, interpolated_y
    
    def calc_yaw_angle(self, s):
        dx = self.sx.calculate_first_derivative(s)
        dy = self.sy.calculate_first_derivative(s)
        yaw_angle = math.atan2(dy, dx)
        return yaw_angle
    
    def calc_curvature(self, s):
        dx = self.sx.calculate_first_derivative(s)
        ddx = self.sx.calculate_second_derivative(s)

        dy = self.sy.calculate_first_derivative(s)
        ddy = self.sy.calculate_second_derivative(s)

        curvature = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2) ** (3/2))
        return curvature

    def _calc_base_points(self, x_points, y_points):
        dx = np.diff(x_points)
        dy = np.diff(y_points)
        self.ds = np.hypot(dx, dy)
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

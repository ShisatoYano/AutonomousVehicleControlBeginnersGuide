"""
cubic_spline_2d.py

Author: Shisato Yano
"""

import numpy as np

from cubic_spline import CubicSpline


class CubicSpline2D:
    """
    2D Cubic Spline class
    Each section between two points are approximated as the following equation
    y_i = a_i + b_i * (x - x_i) + c_i * (x - x_i)^2 + d_i * (x - x_i)^3
    """

    def __init__(self, x_points, y_points):
        print(x_points)
        print(y_points)
    
    def _calculate_distance_from_start(self, x_points, y_points):
        pass

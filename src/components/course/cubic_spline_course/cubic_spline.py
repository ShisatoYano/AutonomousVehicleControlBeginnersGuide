"""
cubic_spline.py

Author: Shisato Yano
"""

import numpy as np


class CubicSpline:
    """
    1D Cubic Spline class
    Each section between two points are approximated as the following equation
    y_i = a_i + b_i * (x - x_i) + c_i * (x - x_i)^2 + d_i * (x - x_i)^3
    """

    def __init__(self, x_points, y_points):
        """
        Constructor
        x_points: List of x coordinate points. This must be stored in ascending order.
        y_points: List of y coordinate points.
        """
        
        h = np.diff(x_points)
        if np.any(h < 0):
            raise ValueError("X coordinate points must be stored in ascending order")
        
        self.a, self.b, self.c, self.d = [], [], [], []
        self.x_points = x_points
        self.y_points = y_points
        self.size_x_points = len(self.x_points)

        self._calculate_coefficient_a()
    
    def _calculate_coefficient_a(self):
        pass

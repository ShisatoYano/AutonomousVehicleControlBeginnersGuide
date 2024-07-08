"""
cubic_spline.py

Author: Shisato Yano
"""

import numpy as np


class CubicSpline:
    """
    1D Cubic Spline class
    """

    def __init__(self, x_points, y_points):
        h = np.diff(x_points)
        if np.any(h < 0):
            raise ValueError("X coordinate points must be stored in ascending order")

"""
xy_array.py

Author: Shisato Yano
"""

from math import sin, cos
import numpy as np


class XYArray:
    """
    X-Y 2D array data and logic class
    """

    def __init__(self, array):
        """
        Constructor
        array: np.array([[x1, x2,..., xn], [y1, y2,..., yn]])
        """

        self.array = array
    
    def rotation(self, angle_rad):
        angle_cos = cos(angle_rad)
        angle_sin = sin(angle_rad)

        rotation_matrix = np.array([[angle_cos, -angle_sin],
                                    [angle_sin, angle_cos]])
        
        rotated_array = rotation_matrix.dot(self.array)

        return XYArray(rotated_array)

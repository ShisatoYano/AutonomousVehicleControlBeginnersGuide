"""
xy_array.py

Author: Shisato Yano
"""

import sys
import numpy as np
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../common")
from matrix_lib import rot_mat_22


class XYArray:
    """
    X-Y 2D array data and logic class
    """

    def __init__(self, data):
        """
        Constructor
        data: np.array([[x1, x2,..., xn], [y1, y2,..., yn]])
        """

        self.data = data
    
    def homogeneous_transformation(self, x, y, angle_rad):
        """
        Function for homogeneous transformation
        x: Amount of x-axis translation
        y: Amount of y-axis translation
        angle_rad: Rotation angle[rad]
        Return transformed XYArray object
        """
        
        rotated_data = rot_mat_22(angle_rad) @ self.data

        translated_data = rotated_data + np.array([[x], [y]])

        return XYArray(translated_data)
    
    def get_data(self):
        """
        Return array data
        Type is ndarray object
        """

        return self.data
    
    def get_x_data(self):
        """
        Return x array data
        Type is ndarray object
        """

        return self.data[0, :]
    
    def get_y_data(self):
        """
        Return y array data
        Type is ndarray object
        """

        return self.data[1, :]
    
    def get_dimension(self):
        """
        Return x-y array data's dimension value
        """

        return self.data.ndim

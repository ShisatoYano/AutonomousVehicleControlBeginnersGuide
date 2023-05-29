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
        
        angle_cos = cos(angle_rad)
        angle_sin = sin(angle_rad)

        rotation_matrix = np.array([[angle_cos, -angle_sin],
                                    [angle_sin, angle_cos]])
        
        rotated_data = rotation_matrix @ self.data

        translated_data = rotated_data + np.ones(rotated_data.shape) * np.array([[x], [y]])

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

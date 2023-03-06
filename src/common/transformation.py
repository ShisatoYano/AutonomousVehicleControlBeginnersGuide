"""
transformation.py

Author: Shisato Yano
"""

from math import sin, cos
import numpy as np


class Transformation:
    """
    Transformation methods collection class
    """

    @staticmethod
    def rotation(points, angle_rad):
        angle_cos = cos(angle_rad)
        angle_sin = sin(angle_rad)

        rotation_matrix = np.array([[angle_cos, -angle_sin],
                                    [angle_sin, angle_cos]])
        
        return rotation_matrix.dot(points)
    
    @staticmethod
    def translation(points, x, y):
        return (points + np.ones(points.shape) * np.array([[x], [y]]))
    
    @staticmethod
    def homogeneous_transformation(points, base_vector):
        x = base_vector[0, 0]
        y = base_vector[1, 0]
        angle_rad = base_vector[2, 0]
        rotated_points = Transformation.rotation(points, angle_rad)
        return Transformation.translation(rotated_points, x, y)

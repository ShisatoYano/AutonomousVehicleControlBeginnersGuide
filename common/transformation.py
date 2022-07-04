"""
Transformation function library

Author: Shisato Yano
"""

from cmath import pi
from math import sin, cos
import numpy as np


def rotate_translate_2d(points, x_m, y_m, angle_deg):
    angle_cos = cos(np.deg2rad(angle_deg))
    angle_sin = sin(np.deg2rad(angle_deg))
    rotation_matrix = np.array([[angle_cos, -angle_sin], [angle_sin, angle_cos]])
    rotated_points = rotation_matrix.dot(points)
    transformed_points = rotated_points + np.ones(points.shape) * (np.array([[x_m], [y_m]]))
    return transformed_points


def limit_angle_pi_2_pi(angle_deg):
    angle_rad = np.deg2rad(angle_deg)
    return (angle_rad + pi) % (2 * pi) - pi

"""
matrix_lib.py

Author: Shisato Yano
"""

import numpy as np
from math import sin, cos


def hom_mat_33(x, y, yaw):
    """
    Homogeneous transformation matrix 3x3
    x: x direction translation[m]
    y: y direction translation[m]
    yaw: yaw direction rotation[rad]
    """

    cos_yaw, sin_yaw = cos(yaw), sin(yaw)

    return np.array([[cos_yaw, -sin_yaw, x],
                     [sin_yaw, cos_yaw, y],
                     [0.0, 0.0, 1.0]])


def rot_mat_22(yaw):
    cos_yaw, sin_yaw = cos(yaw), sin(yaw)

    return np.array([[cos_yaw, -sin_yaw],
                     [sin_yaw, cos_yaw]])

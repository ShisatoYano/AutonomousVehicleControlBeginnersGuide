"""
angle_lib.py

Author: Shisato Yano
"""

import numpy as np


def pi_to_pi(angle_rad):
    """
    Function to limit angle[rad] to [-pi, pi)
    angle_rad: Original angle[rad] of any size, a float or a numpy array
    """

    # shift by pi so that the modulo wraps into [0, 2pi), then shift back
    return (angle_rad + np.pi) % (2 * np.pi) - np.pi


def zero_to_2pi(angle_rad):
    """
    Function to limit angle[rad] to [0, 2pi)
    angle_rad: Original angle[rad] of any size, a float or a numpy array
    """

    return angle_rad % (2 * np.pi)

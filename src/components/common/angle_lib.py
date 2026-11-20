"""
angle_lib.py

Author: Shisato Yano
"""

import numpy as np


def pi_to_pi(angle_rad):
    """
    Function to limit angle[rad] to the range [-pi, pi)
    Works for any angle (not only within one turn) and for numpy arrays
    angle_rad: Original angle[rad], scalar or numpy array
    """

    return (angle_rad + np.pi) % (2.0 * np.pi) - np.pi


def zero_to_2pi(angle_rad):
    """
    Function to limit angle[rad] to the range [0, 2*pi)
    Works for any angle and for numpy arrays
    angle_rad: Original angle[rad], scalar or numpy array
    """

    return angle_rad % (2.0 * np.pi)

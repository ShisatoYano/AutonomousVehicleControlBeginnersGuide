"""
angle_lib.py

Author: Shisato Yano
"""

import numpy as np


def _as_angle(angle_rad):
    return np.asarray(angle_rad)


def pi_to_pi(angle_rad):
    """
    Function to limit angle[rad] between -pi and pi
    angle_rad: Original angle[rad]. Scalar or numpy array.
    """

    wrapped = (_as_angle(angle_rad) + np.pi) % (2.0 * np.pi) - np.pi
    if np.isscalar(angle_rad):
        return wrapped.item()
    return wrapped


def zero_to_2pi(angle_rad):
    """
    Function to limit angle[rad] between 0 and 2pi
    angle_rad: Original angle[rad]. Scalar or numpy array.
    """

    wrapped = _as_angle(angle_rad) % (2.0 * np.pi)
    if np.isscalar(angle_rad):
        return wrapped.item()
    return wrapped

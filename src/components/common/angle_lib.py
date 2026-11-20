"""
angle_lib.py

Author: Shisato Yano
"""

import numpy as np
import numpy.typing as npt


def pi_to_pi(angle_rad: float | npt.NDArray[np.float64]) -> float | npt.NDArray[np.float64]:
    """
    Function to limit angle[rad] between -pi and pi
    Any angle is accepted, also one several turns away like 5 * pi, and pi itself becomes -pi.
    angle_rad: Original angle[rad], a scalar or a numpy array of any shape
    """

    return (angle_rad + np.pi) % (2 * np.pi) - np.pi


def zero_to_2pi(angle_rad: float | npt.NDArray[np.float64]) -> float | npt.NDArray[np.float64]:
    """
    Function to limit angle[rad] between 0 and 2 * pi
    Any angle is accepted, also a negative one or one several turns away.
    angle_rad: Original angle[rad], a scalar or a numpy array of any shape
    """

    return angle_rad % (2 * np.pi)

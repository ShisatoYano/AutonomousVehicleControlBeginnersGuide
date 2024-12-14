"""
angle_lib.py

Author: Shisato Yano
"""

import numpy as np


def pi_to_pi(angle_rad):
    """
    Function to limit angle[rad] between -pi and pi
    angle_rad: Original angle[rad]
    """
    
    modified_angle_rad = angle_rad
    
    while angle_rad >= np.pi:
        modified_angle_rad -= 2 * np.pi
    
    while angle_rad < -np.pi:
        modified_angle_rad += 2 * np.pi
    
    return modified_angle_rad

"""
extended_kalman_filter_localizer.py

Author: Shisato Yano
"""

import sys
import numpy as np
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../../state")
from state import State

class ExtendedKalmanFilterLocalizer:
    """
    Self localization by Extended Kalman Filter class
    """

    def __init__(self, state, accel_noise_std=1.0, yaw_rate_noise=30.0, 
                 color='r'):
        """
        Constructor
        state: Vehicle's state object
        color: Color of drawing error covariance ellipse
        """
        
        self.state = state
        self.INPUT_NOISE_VAR_MAT = np.diag([accel_noise_std, np.deg2rad(yaw_rate_noise)]) ** 2
        self.DRAW_COLOR = color
    
    def update(self, accel_mps2, yaw_rate_rps, time_s, gnss):
        print(gnss)

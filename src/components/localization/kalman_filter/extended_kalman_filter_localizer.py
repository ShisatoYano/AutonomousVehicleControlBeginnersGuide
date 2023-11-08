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

    def __init__(self, state, accel_noise=1.0, yaw_rate_noise=50.0, 
                 color='r'):
        """
        Constructor
        state: Vehicle's state object
        color: Color of drawing error covariance ellipse
        """
        
        self.state = state
        self.INPUT_NOISE_VAR_MAT = np.diag([accel_noise, np.deg2rad(yaw_rate_noise)]) ** 2
        self.DRAW_COLOR = color
    
    def update(self, state, accel_mps2, yaw_rate_rps, time_s, gnss):
        last_state = np.array([[state.get_x_m()],
                               [state.get_y_m()],
                               [state.get_yaw_rad()],
                               [state.get_speed_mps()]])
        
        input_org = np.array([[accel_mps2],
                              [yaw_rate_rps]])
        input_noise = self.INPUT_NOISE_VAR_MAT @ np.random.randn(2, 1)
        next_input = input_org + input_noise
        
        next_state = State.motion_model(last_state, next_input, time_s)

        return next_state

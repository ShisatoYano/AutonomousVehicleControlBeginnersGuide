"""
extended_kalman_filter_localizer.py

Author: Shisato Yano
"""

import sys
import numpy as np
from pathlib import Path
from math import cos, sin

sys.path.append(str(Path(__file__).absolute().parent) + "/../../state")
from state import State

class ExtendedKalmanFilterLocalizer:
    """
    Self localization by Extended Kalman Filter class
    """

    def __init__(self, state, accel_noise=1.0, yaw_rate_noise=30.0, 
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
        
        # predict
        input_org = np.array([[accel_mps2],
                              [yaw_rate_rps]])
        input_noise = self.INPUT_NOISE_VAR_MAT @ np.random.randn(2, 1)
        next_input = input_org + input_noise
        
        pred_state = State.motion_model(last_state, next_input, time_s)
        jF = self._jacobian_F(pred_state, next_input, time_s)
        jG = self._jacobian_G(pred_state, time_s)

        return pred_state
    
    def _jacobian_F(self, state, input, time_s):
        yaw = state[2, 0]
        spd = state[3, 0]
        acl = input[0, 0]
        t = time_s

        sin_yaw = sin(yaw)
        cos_yaw = cos(yaw)

        jF = np.array([[1, 0, -spd*sin_yaw*t-acl*sin_yaw*t**2/2, cos_yaw*t],
                       [0, 1, spd*cos_yaw*t+acl*cos_yaw*t**2/2, sin_yaw*t],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])
        
        return jF
    
    def _jacobian_G(self, state, time_s):
        yaw = state[2, 0]
        t = time_s

        jG = np.array([[cos(yaw) * t / 2, 0],
                       [sin(yaw) * t / 2, 0],
                       [0, t],
                       [t, 0]])
        
        return jG

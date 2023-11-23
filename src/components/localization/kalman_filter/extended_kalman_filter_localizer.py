"""
extended_kalman_filter_localizer.py

Author: Shisato Yano
"""

import sys
import numpy as np
from pathlib import Path
from math import cos, sin, sqrt, atan2

sys.path.append(str(Path(__file__).absolute().parent) + "/../../state")
from state import State

class ExtendedKalmanFilterLocalizer:
    """
    Self localization by Extended Kalman Filter class
    """

    def __init__(self, accel_noise=1.0, yaw_rate_noise=30.0, color='r'):
        """
        Constructor
        color: Color of drawing error covariance ellipse
        """
        
        self.state = np.zeros((4, 1))
        self.cov_mat = np.eye(4)
        self.INPUT_NOISE_VAR_MAT = np.diag([accel_noise, np.deg2rad(yaw_rate_noise)]) ** 2
        self.DRAW_COLOR = color
    
    def update(self, state, accel_mps2, yaw_rate_rps, time_s, gnss):
        last_state = np.array([[state.get_x_m()],
                               [state.get_y_m()],
                               [state.get_yaw_rad()],
                               [state.get_speed_mps()]])
        
        # input with noise
        input_org = np.array([[accel_mps2],
                              [yaw_rate_rps]])
        input_noise = self.INPUT_NOISE_VAR_MAT @ np.random.randn(2, 1)
        next_input = input_org + input_noise
        
        # predict
        pred_state = State.motion_model(last_state, next_input, time_s)
        jF = self._jacobian_F(pred_state, next_input, time_s)
        jG = self._jacobian_G(pred_state, time_s)
        Q = self.INPUT_NOISE_VAR_MAT
        last_cov = self.cov_mat
        pred_cov = jF @ last_cov @jF.T + jG @ Q @ jG.T

        self.state = pred_state
        self.cov_mat = pred_cov

        return pred_state
    
    def draw(self, axes, elems, pose):
        eig_val, eig_vec = np.linalg.eig(self.cov_mat)
        if eig_val[0] >= eig_val[1]: big_idx, small_idx = 0, 1
        else: big_idx, small_idx = 1, 0
        a, b = sqrt(3.0 * eig_val[big_idx]), sqrt(3.0 * eig_val[small_idx])
        angle = atan2(eig_vec[1, big_idx], eig_vec[0, big_idx])

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

        jG = np.array([[cos(yaw) * t**2 / 2, 0],
                       [sin(yaw) * t**2 / 2, 0],
                       [0, t],
                       [t, 0]])
        
        return jG

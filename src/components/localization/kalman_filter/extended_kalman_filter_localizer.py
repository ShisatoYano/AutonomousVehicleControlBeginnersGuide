"""
extended_kalman_filter_localizer.py

Author: Shisato Yano
"""

import sys
import numpy as np
from pathlib import Path
from math import cos, sin

sys.path.append(str(Path(__file__).absolute().parent) + "/../../state")
sys.path.append(str(Path(__file__).absolute().parent) + "/../../array")
sys.path.append(str(Path(__file__).absolute().parent) + "/../../sensors/gnss")
sys.path.append(str(Path(__file__).absolute().parent) + "/../../common")
from state import State
from plot_lib import draw_covariance_ellipse


class ExtendedKalmanFilterLocalizer:
    """
    Self localization by Extended Kalman Filter class
    """

    def __init__(self, accel_noise=0.2, yaw_rate_noise=10.0,
                 obsrv_x_noise=1.0, obsrv_y_noise=1.0, color='r'):
        """
        Constructor
        color: Color of drawing error covariance ellipse
        """
        
        self.state = np.zeros((4, 1))
        self.cov_mat = np.eye(4)
        self.INPUT_NOISE_VAR_MAT = np.diag([accel_noise, np.deg2rad(yaw_rate_noise)]) ** 2
        self.OBSRV_NOISE_VAR_MAT = np.diag([obsrv_x_noise, obsrv_y_noise]) ** 2
        self.JACOB_H = np.array([[1, 0, 0, 0],
                                 [0, 1, 0, 0]])
        self.DRAW_COLOR = color
    
    def update(self, state, accel_mps2, yaw_rate_rps, time_s, gnss):
        """
        Function to update data
        state: Last extimated state data
        accel_mps2: Acceleration input from controller
        yaw_rate_rps: Yaw rate input from controller
        time_s: Simulation interval time[sec]
        Return: Estimated state data
        """
        
        last_state = np.array([[state.get_x_m()],
                               [state.get_y_m()],
                               [state.get_yaw_rad()],
                               [state.get_speed_mps()]])
        
        # input with noise
        input_org = np.array([[accel_mps2],
                              [yaw_rate_rps]])
        input_noise = self.INPUT_NOISE_VAR_MAT @ np.random.randn(2, 1)
        next_input = input_org + input_noise
        
        # predict state
        pred_state = State.motion_model(last_state, next_input, time_s)
        jF = self._jacobian_F(pred_state, next_input, time_s)
        jG = self._jacobian_G(pred_state, time_s)
        Q = self.INPUT_NOISE_VAR_MAT
        last_cov = self.cov_mat
        pred_cov = jF @ last_cov @jF.T + jG @ Q @ jG.T

        # predict observation
        pred_obsrv = self._observation_model(pred_state)
        jH = self.JACOB_H
        R = self.OBSRV_NOISE_VAR_MAT
        pred_obsrv_cov = jH @ pred_cov @ jH.T + R
        
        # kalman gain
        k = pred_cov @ jH.T @ np.linalg.inv(pred_obsrv_cov)

        # update
        inov = gnss - pred_obsrv
        est_state = pred_state + k @ inov
        est_cov = pred_cov - k @ pred_obsrv_cov @ k.T

        self.state = est_state
        self.cov_mat = est_cov

        return est_state
    
    def draw(self, axes, elems, pose):
        """
        Function to draw error covariance ellipse
        axes: Axes objects of figure
        elems: List of plot object
        pose: Vehicle's pose[x, y, yaw]
        """
        
        # ellipse of the x-y block of the 4x4 state covariance
        draw_covariance_ellipse(axes, elems, pose[0, 0], pose[1, 0], self.cov_mat[:2, :2],
                                color=self.DRAW_COLOR)

    def _jacobian_F(self, state, input, time_s):
        """
        Private function to calculate jacobian F
        state: Predicted state
        input: Input vector[accel, yaw rate] from controller
        time_s: Simulation interval time[sec]
        Return: Jacobian F
        """
        
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
        """
        Private function to calculate jacobian G
        state: Predicted state
        time_s: Simulation interval time[sec]
        Return: Jacobian G
        """
        
        yaw = state[2, 0]
        t = time_s

        jG = np.array([[cos(yaw) * t**2 / 2, 0],
                       [sin(yaw) * t**2 / 2, 0],
                       [0, t],
                       [t, 0]])
        
        return jG
    
    def _observation_model(self, state):
        """
        Private function of observation model
        state: Predicted state
        Return Predicted observation data
        """
        
        H = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0]])
        
        x = np.array([[state[0, 0]],
                      [state[1, 0]],
                      [state[2, 0]],
                      [state[3, 0]]])
        
        return H @ x

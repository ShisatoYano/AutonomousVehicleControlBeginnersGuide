"""
unscented_kalman_filter_localizer.py

Author: Bruno DOKPOMIWA
"""

import sys
import numpy as np
import scipy.linalg as spl
from pathlib import Path
from math import cos, sin, sqrt, atan2, pi

sys.path.append(str(Path(__file__).absolute().parent) + "/../../state")
sys.path.append(str(Path(__file__).absolute().parent) + "/../../array")
sys.path.append(str(Path(__file__).absolute().parent) + "/../../sensors/gnss")
from state import State
from xy_array import XYArray


class UnscentedKalmanFilterLocalizer:
    """
    Self localization by Unscented Kalman Filter class
    """

    def __init__(self, accel_noise=0.2, yaw_rate_noise=10.0,
                 obsrv_x_noise=1.0, obsrv_y_noise=1.0,
                 alpha=0.001, beta=2, kappa=0, color='r'):
        """
        Constructor
        accel_noise: Standard deviation of acceleration noise
        yaw_rate_noise: Standard deviation of yaw rate noise[deg]
        obsrv_x_noise: Standard deviation of x observation noise
        obsrv_y_noise: Standard deviation of y observation noise
        alpha: UKF's alpha parameter (controls spread of sigma points)
        beta: UKF's beta parameter (incorporates prior knowledge)
        kappa: UKF's kappa parameter (secondary scaling parameter)
        color: Color of drawing error covariance ellipse
        """
        
        # state dimension (x, y, yaw, speed)
        self.DIM_NUM = 4
        
        # UKF parameters
        self.ALPHA = alpha
        self.BETA = beta
        self.KAPPA = kappa
        self._decide_sigma_weights()
        
        # noise covariance matrices
        self.INPUT_NOISE_VAR_MAT = np.diag([accel_noise, np.deg2rad(yaw_rate_noise)]) ** 2
        self.OBSRV_NOISE_VAR_MAT = np.diag([obsrv_x_noise, obsrv_y_noise]) ** 2
        
        # initialize state and covariance
        self.state = np.zeros((self.DIM_NUM, 1))
        self.cov_mat = np.eye(self.DIM_NUM)
        
        self.DRAW_COLOR = color
    
    def _decide_sigma_weights(self):
        """
        Private function to decide weights for each sigma points
        """
        
        self.LAMBDA = self.ALPHA**2 * (self.DIM_NUM + self.KAPPA) - self.DIM_NUM
        
        # for 2n + 1 sigma points
        state_weights, cov_weights = [], []
        DIM_LAMBDA = self.DIM_NUM + self.LAMBDA
        state_weights.append(self.LAMBDA / DIM_LAMBDA)  # i = 0
        cov_weights.append((self.LAMBDA / DIM_LAMBDA) + (1 - self.ALPHA**2 + self.BETA))  # i = 0
        for i in range(2 * self.DIM_NUM):
            state_weights.append(1 / (2 * DIM_LAMBDA))
            cov_weights.append(1 / (2 * DIM_LAMBDA))
        self.STATE_WEIGHTS = np.array([state_weights])
        self.COV_WEIGHTS = np.array([cov_weights])
        
        # for generating sigma points
        self.GAMMA = sqrt(DIM_LAMBDA)
    
    def _generate_sigma_points(self, state, cov):
        """
        Private function to generate sigma points
        state: state vector (x, y, yaw, speed)
        cov: covariance matrix 4x4
        Return: array of sigma points (4 x 9)
        """
        
        sigmas = state
        cov_sqr = spl.sqrtm(cov)  # matrix square root
        
        # add each dimension's std to state vector
        # positive direction
        for i in range(self.DIM_NUM):
            sigmas = np.hstack((sigmas, state + self.GAMMA * cov_sqr[:, i:i+1]))
        # negative direction
        for i in range(self.DIM_NUM):
            sigmas = np.hstack((sigmas, state - self.GAMMA * cov_sqr[:, i:i+1]))
        return sigmas
    
    def _predict_sigmas_motion(self, sigmas, input, time_s):
        """
        Private function to predict motion of sigma points
        sigmas: array of sigma points
        input: input vector [accel, yaw_rate]
        time_s: simulation interval time[sec]
        Return: array of motion predicted sigma points
        """
        
        pred_sigmas = np.zeros_like(sigmas)
        for i in range(sigmas.shape[1]):
            pred_sigmas[:, i:i+1] = State.motion_model(sigmas[:, i:i+1], input, time_s)
        return pred_sigmas
    
    def _predict_state_covariance(self, pred_state, pred_sigmas):
        """
        Private function to predict state covariance
        pred_state: predicted state vector
        pred_sigmas: array of motion predicted sigma points
        Return: predicted state covariance matrix
        """
        
        diff = pred_sigmas - pred_state
        pred_cov = np.zeros((4, 4))
        for i in range(pred_sigmas.shape[1]):
            pred_cov = pred_cov + self.COV_WEIGHTS[0, i] * diff[:, i:i+1] @ diff[:, i:i+1].T
        return pred_cov
    
    def _observation_model(self, state):
        """
        Private function of observation model
        state: state vector (x, y, yaw, speed)
        Return: predicted observation data (x, y)
        """
        
        return np.array([[state[0, 0]], [state[1, 0]]])
    
    def _predict_sigmas_observation(self, sigmas):
        """
        Private function to predict observation at each sigma point
        sigmas: array of sigma points
        Return: predicted observation vector at each sigma point (2 x 9)
        """
        
        sigmas_num = sigmas.shape[1]
        obv_sigmas = np.zeros((2, sigmas_num))
        for i in range(sigmas_num):
            obv_sigmas[:, i:i+1] = self._observation_model(sigmas[:, i:i+1])
        return obv_sigmas
    
    def _predict_observation_covariance(self, pred_obv, pred_obv_sigmas, obsrv_noise_cov):
        """
        Private function to predict observation covariance
        pred_obv: predicted observation vector
        pred_obv_sigmas: predicted observation at each sigma point
        obsrv_noise_cov: observation noise covariance matrix
        Return: predicted observation covariance matrix
        """
        
        diff = pred_obv_sigmas - pred_obv
        pred_obv_cov = obsrv_noise_cov
        for i in range(pred_obv_sigmas.shape[1]):
            pred_obv_cov = pred_obv_cov + self.COV_WEIGHTS[0, i] * diff[:, i:i+1] @ diff[:, i:i+1].T
        return pred_obv_cov
    
    def _calculate_correlation_matrix(self, pred_state, pred_sigmas, pred_obv, pred_obv_sigmas):
        """
        Private function to calculate correlation matrix between state and observation
        pred_state: predicted state vector
        pred_sigmas: motion predicted sigma points
        pred_obv: predicted observation vector
        pred_obv_sigmas: predicted observation at each sigma point
        Return: correlation matrix between state and observation
        """
        
        sigmas_num = pred_sigmas.shape[1]
        diff_state = pred_sigmas - pred_state
        diff_obv = pred_obv_sigmas - pred_obv
        corr_mat = np.zeros((diff_state.shape[0], diff_obv.shape[0]))
        for i in range(sigmas_num):
            corr_mat = corr_mat + self.COV_WEIGHTS[0, i] * diff_state[:, i:i+1] @ diff_obv[:, i:i+1].T
        return corr_mat
    
    def update(self, state, accel_mps2, yaw_rate_rps, time_s, gnss):
        """
        Function to update data
        state: Last estimated state data
        accel_mps2: Acceleration input from controller
        yaw_rate_rps: Yaw rate input from controller
        time_s: Simulation interval time[sec]
        gnss: GNSS observation data (x, y)
        Return: Estimated state data
        """
        
        last_state = np.array([[state.get_x_m()],
                               [state.get_y_m()],
                               [state.get_yaw_rad()],
                               [state.get_speed_mps()]])
        
        # input with noise
        input_org = np.array([[accel_mps2],
                              [yaw_rate_rps]])
        input_noise = np.sqrt(self.INPUT_NOISE_VAR_MAT) @ np.random.randn(2, 1)
        next_input = input_org + input_noise
        
        # augment state with input noise for sigma point generation
        # create augmented state [x, y, yaw, speed, accel_noise, yaw_rate_noise]
        aug_state = np.vstack((last_state, np.zeros((2, 1))))
        aug_cov = np.block([[self.cov_mat, np.zeros((4, 2))],
                            [np.zeros((2, 4)), self.INPUT_NOISE_VAR_MAT]])
        
        # generate sigma points from augmented state
        aug_sigmas = self._generate_sigma_points(aug_state, aug_cov)
        
        # extract state and input noise from augmented sigma points
        state_sigmas = aug_sigmas[:4, :]
        input_noise_sigmas = aug_sigmas[4:, :]
        
        # predict motion of sigma points with input noise
        pred_sigmas = np.zeros((4, aug_sigmas.shape[1]))
        for i in range(aug_sigmas.shape[1]):
            input_with_noise = next_input + input_noise_sigmas[:, i:i+1]
            pred_sigmas[:, i:i+1] = State.motion_model(state_sigmas[:, i:i+1], input_with_noise, time_s)
        
        # compute predicted state mean
        pred_state = (self.STATE_WEIGHTS @ pred_sigmas.T).T
        
        # compute predicted state covariance
        pred_cov = self._predict_state_covariance(pred_state, pred_sigmas)
        
        # predict observation - generate new sigma points from predicted state and covariance
        obv_state_sigmas = self._generate_sigma_points(pred_state, pred_cov)
        pred_obv_sigmas = self._predict_sigmas_observation(obv_state_sigmas)
        pred_obv = (self.STATE_WEIGHTS @ pred_obv_sigmas.T).T
        
        # compute predicted observation covariance
        pred_obv_cov = self._predict_observation_covariance(pred_obv, pred_obv_sigmas, self.OBSRV_NOISE_VAR_MAT)
        
        # compute correlation matrix between state and observation
        corr_mat = self._calculate_correlation_matrix(pred_state, obv_state_sigmas, pred_obv, pred_obv_sigmas)
        
        # kalman gain
        k = corr_mat @ np.linalg.inv(pred_obv_cov)
        
        # update
        inov = gnss - pred_obv
        est_state = pred_state + k @ inov
        est_cov = pred_cov - k @ pred_obv_cov @ k.T
        
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
        
        # extract 2x2 covariance for x and y
        xy_cov = self.cov_mat[:2, :2]
        eig_val, eig_vec = np.linalg.eig(xy_cov)
        if eig_val[0] >= eig_val[1]: 
            big_idx, small_idx = 0, 1
        else: 
            big_idx, small_idx = 1, 0
        a, b = sqrt(3.0 * eig_val[big_idx]), sqrt(3.0 * eig_val[small_idx])
        angle = atan2(eig_vec[1, big_idx], eig_vec[0, big_idx])

        t = np.arange(0, 2 * pi + 0.1, 0.1)
        xs = [a * cos(it) for it in t]
        ys = [b * sin(it) for it in t]
        xys = np.array([xs, ys])
        xys_array = XYArray(xys)

        transformed_xys = xys_array.homogeneous_transformation(pose[0, 0], pose[1, 0], angle)
        elip_plot, = axes.plot(transformed_xys.get_x_data(), transformed_xys.get_y_data(), color=self.DRAW_COLOR)
        elems.append(elip_plot)


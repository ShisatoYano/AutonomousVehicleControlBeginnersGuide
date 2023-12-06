"""
ukf_ext_params_calibrator.py

Author: Shisato Yano
"""

import numpy as np
import scipy.linalg as spl
from math import sqrt


class UkfExtParamsCalibrator:
    """
    LiDAR's extrinsic parameters calibration by Unscented Kalman Filter
    """

    def __init__(self, alpha=0.001, beta=2, kappa=0,
                 sys_lon_std=0.1, sys_lat_std=0.1, sys_yaw_std=0.1,
                 obv_x_std=0.5, obv_y_std=0.5, obv_yaw_std=0.5):
        """
        Constructor
        alpha: number of UKF's alpha parameter
        beta: number of UKF's beta parameter
        kappa: number of UKF's kappa parameter
        sys_lon_std: standard deviation of longitudianl system noise[m]
        sys_lat_std: standard deviation of lateral system noise[m]
        sys_yaw_std: standard deviation of yaw direction system noise[deg]
        obv_x_std: standard deviation of x direction observation noise[m]
        obv_y_std: standard deviation of y direction observation noise[m]
        obv_yaw_std: standard deviation of yaw direction observation noise[deg]
        """

        # parameters for deciding number of sigma points and those weights
        self.DIM_NUM = 3
        self.ALPHA = alpha
        self.BETA = beta
        self.KAPPA = kappa
        self._decide_sigma_weights()

        # parameters of system/observation noise covariance
        self.SYS_NOISE = np.diag([sys_lon_std, sys_lat_std, np.deg2rad(sys_yaw_std)]) ** 2 # system noise
        self.OBV_NOISE = np.diag([obv_x_std, obv_y_std, np.deg2rad(obv_yaw_std)]) ** 2 # observation noise

        # calibrated result
        self.state = np.zeros((self.DIM_NUM, 1)) # state vector on vehicle coordinate
        self.cov = np.eye(self.DIM_NUM) # covariance matrix of each state variables
    
    def _decide_sigma_weights(self):
        """
        Private function to decide weights for each sigma points
        """
        
        self.LAMBDA = self.ALPHA**2 * (self.DIM_NUM + self.KAPPA) - self.DIM_NUM

        # for 2n + 1 sigma points
        state_weights, cov_weights = [], []
        DIM_LAMBDA = self.DIM_NUM + self.LAMBDA
        state_weights.append(self.LAMBDA / DIM_LAMBDA) # i = 0
        cov_weights.append((self.LAMBDA / DIM_LAMBDA) + (1 - self.ALPHA**2 + self.BETA)) # i = 0
        for i in range(2 * self.DIM_NUM):
            state_weights.append(1 / (2 * DIM_LAMBDA))
            cov_weights.append(1 / (2 * DIM_LAMBDA))
        self.STATE_WEIGHTS = np.array([state_weights])
        self.COV_WEIGHTS = np.array([cov_weights])
        
        # for generating sigma points
        self.GAMMA = sqrt(DIM_LAMBDA)
    
    def _generate_sigme_points(self, state, cov):
        """
        Private function to generate sigma points
        state: state vector (lon, lat, yaw)
        cov: covariance matrix 3x3
        Return: array of sigma points
        """
        
        sigmas = state
        cov_sqr = spl.sqrtm(cov) # standard deviation matrix
        
        # add each dimension's std to state vector
        # positive direction
        for i in range(self.DIM_NUM):
            sigmas = np.hstack((sigmas, state + self.GAMMA * cov_sqr[:, i:i+1]))
        # negative direction
        for i in range(self.DIM_NUM):
            sigmas = np.hstack((sigmas, state - self.GAMMA * cov_sqr[:, i:i+1]))
        return sigmas

    def _motion_model(self, state):
        """
        Private function of motion model
        state: state vector (lon, lat, yaw)
        Return: state vector (lon, lat, yaw)
        """
        
        # this algorithm's purpose to estimate parameters
        # those don't change overtime
        # so, this motion model doesn't need to have input part
        A = np.array([[1.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0],
                      [0.0, 0.0, 1.0]])
        return A @ state

    def _predict_sigmas_motion(self, sigmas):
        """
        Private function to predict overtime motion of sigma points
        sigmas: array of sigma points
        Return: array of moton predicted sigma points
        """
        
        for i in range(sigmas.shape[1]):
            sigmas[:, i:i+1] = self._motion_model(sigmas[:, i:i+1])
        return sigmas

    def _predict_covariance(self, pred_state, pred_sigmas, noise):
        """
        Private function to predict motion/observation covariance
        pred_state: predicted state vector (lon, lat, yaw)
        pred_sigmas: array of motion predicted sigma points
        noise: parameters of system/observation noise covariance
        Return: matrix of predicted system/observation covariance
        """
        
        diff = pred_sigmas - pred_state[0:pred_sigmas.shape[0]]
        pred_cov = noise
        for i in range(pred_sigmas.shape[1]):
            pred_cov = pred_cov + self.COV_WEIGHTS[0, i] * diff[:, i:i+1] @ diff[:, i:i+1].T
        return pred_cov

    def calibrate_extrinsic_params(self, sensor_odom_tf, vehicle_odom_tf):
        """
        sensor_odom_tf: odometry of sensor expressed as 3x3 homogeneous transformation matrix
        vehicle_odom_tf: odometry of vehicle expressed as 3x3 homogeneous transformation matrix
        """
        
        # last updated state and covariance
        last_state = self.state
        last_cov = self.cov

        # predict state
        sigmas = self._generate_sigme_points(last_state, last_cov)
        pred_sigmas = self._predict_sigmas_motion(sigmas)
        pred_state = (self.STATE_WEIGHTS @ pred_sigmas.T).T
        pred_cov = self._predict_covariance(pred_state, pred_sigmas, self.SYS_NOISE)

        # predict observation
        sigmas = self._generate_sigme_points(pred_state, pred_cov)
        pred_obv_sigmas = self._predict_sigmas_observation(sigmas, vehicle_odom_tf)
        pred_obv = (self.STATE_WEIGHTS @ pred_obv_sigmas.T).T
        pred_obv_cov = self._predict_covariance(pred_obv, pred_obv_sigmas, self.OBV_NOISE)
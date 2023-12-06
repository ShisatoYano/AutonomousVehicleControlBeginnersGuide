"""
ukf_ext_params_calibrator.py

Author: Shisato Yano
"""

import numpy as np
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

"""
sensor_parameters.py

Author: Shisato Yano
"""

import numpy as np
import scipy.linalg as spl
import sys
from pathlib import Path

from math import sin, cos, sqrt

sys.path.append(str(Path(__file__).absolute().parent) + "/../array")
from xy_array import XYArray


class SensorParameters:
    """
    Parameters class for sensor
    """

    def __init__(self, lon_m=0.0, lat_m=0.0, min_m=0.5, max_m=40, reso_deg=2.0,
                 angle_std_scale=0.01, dist_std_rate=0.005):
        """
        Constructor
        lon_m: longitudinal installation position on vehicle coordinate system[m]
        lat_m: lateral installation position on vehicle coordinate system[m]
        min_m: minimum sensing range[m]
        max_m: maximum sensing range[m]
        reso_deg: resolution of sensing angle[deg]
        angle_std_scale: scale of angle's standard deviation
        dist_std_rate: rate of distance's standard deviation
        """
        
        self.INST_LON_M = lon_m
        self.INST_LAT_M = lat_m

        self.MIN_RANGE_M = min_m
        self.MAX_RANGE_M = max_m

        self.RESO_RAD = np.deg2rad(reso_deg)

        self.ANGLE_STD_SCALE = angle_std_scale
        self.DIST_STD_RATE = dist_std_rate

        self.inst_pos_array = XYArray(np.array([[self.INST_LON_M], [self.INST_LAT_M]]))
        self.global_x_m = None
        self.global_y_m = None

        self.DIM_NUM = 3 # state vector[lon, lat, yaw]
        self.ALPHA = 0.001
        self.BETA = 2
        self.KAPPA = 0
        self._decide_sigma_weights()
        self.est_inst_array = np.zeros((self.DIM_NUM, 1))
        self.prev_sensor_tf = np.zeros((self.DIM_NUM, self.DIM_NUM))
        self.curr_sensor_tf = np.zeros((self.DIM_NUM, self.DIM_NUM))
        self.prev_vehicle_tf = np.zeros((self.DIM_NUM, self.DIM_NUM))
        self.curr_vehicle_tf = np.zeros((self.DIM_NUM, self.DIM_NUM))
        self.first_sensor_pos = True
        self.first_vehicle_pos = True
        self.state = np.zeros((self.DIM_NUM, 1)) # estimated state vector
        self.cov = np.eye(self.DIM_NUM) # estimated covariance matrix
    
    def calculate_global_pos(self, state):
        """
        Function to calculate sensor's installation position on global coordinate system
        state: vehicle's state object
        """

        pose = state.x_y_yaw()
        transformed_array = self.inst_pos_array.homogeneous_transformation(pose[0, 0], pose[1, 0], pose[2, 0])
        
        self.global_x_m = transformed_array.get_x_data()
        self.global_y_m = transformed_array.get_y_data()

        # convert sensor's global pose to homegeneous transformation matrix
        if self.first_sensor_pos:
            self.prev_sensor_tf = self._hom_mat(self.global_x_m[0], self.global_y_m[0], pose[2, 0])
            self.curr_sensor_tf = self.prev_sensor_tf
            self.first_sensor_pos = False
        else:
            self.prev_sensor_tf = self.curr_sensor_tf
            self.curr_sensor_tf = self._hom_mat(self.global_x_m[0], self.global_y_m[0], pose[2, 0])
    
    def _hom_mat(self, x, y, yaw):
        cos_yaw = cos(yaw)
        sin_yaw = sin(yaw)
        
        mat = np.array([[cos_yaw, -sin_yaw, x],
                        [sin_yaw, cos_yaw, y],
                        [0.0, 0.0, 1.0]])
        
        return mat

    def _decide_sigma_weights(self):
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

    def _generate_sigme_points(self):
        sigmas = self.state
        cov_sqr = spl.sqrtm(self.cov) # standard deviation
        
        # add each dimension's std to state vector
        # positive direction
        for i in range(self.DIM_NUM):
            sigmas = np.hstack((sigmas, self.state + self.GAMMA * cov_sqr[:, i:i+1]))
        # negative direction
        for i in range(self.DIM_NUM):
            sigmas = np.hstack((sigmas, self.state - self.GAMMA * cov_sqr[:, i:i+1]))
        return sigmas

    def estimate_extrinsic_params(self, state):
        # current vehicle pose
        pose = state.x_y_yaw()

        # sensor odometry between 2 steps
        sensor_odom_tf = np.linalg.inv(self.prev_sensor_tf) @ self.curr_sensor_tf

        # vehicle odometry between 2 steps
        if self.first_vehicle_pos:
            self.prev_vehicle_tf = self._hom_mat(pose[0, 0], pose[1, 0], pose[2, 0])
            self.curr_vehicle_tf = self.prev_vehicle_tf
            self.first_vehicle_pos = False
        else:
            self.prev_vehicle_tf = self.curr_vehicle_tf
            self.curr_vehicle_tf = self._hom_mat(pose[0, 0], pose[1, 0], pose[2, 0])
        vehicle_odom_tf = np.linalg.inv(self.prev_vehicle_tf) @ self.curr_vehicle_tf

        # predict
        sigmas = self._generate_sigme_points()

    def get_global_x_m(self):
        """
        Getter of sensor's x installation position on global coordinate system
        """

        return self.global_x_m[0]
    
    def get_global_y_m(self):
        """
        Getter of sensor's y installation position on global coordinate system
        """

        return self.global_y_m[0]
    
    def draw_pos(self, axes, elems):
        """
        Function to draw sensor's installation position on vehicle
        axes: axes object of figure
        elems: list of plot object
        """

        pos_plot, = axes.plot(self.global_x_m, self.global_y_m, marker='.', color='b')
        elems.append(pos_plot)

        elems.append(axes.text(self.global_x_m, self.global_y_m + 3,
                               "Sensor Lon Est:{0:.2f}/True:{1:.2f}[m]".format(self.est_inst_array[0, 0], self.INST_LON_M),
                               fontsize=10))
        elems.append(axes.text(self.global_x_m, self.global_y_m + 2.5,
                               "Sensor Lat Est:{0:.2f}/True:{1:.2f}[m]".format(self.est_inst_array[1, 0], self.INST_LAT_M),
                               fontsize=10))

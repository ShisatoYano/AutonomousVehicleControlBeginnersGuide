"""
sensor_parameters.py

Author: Shisato Yano
"""

import numpy as np
import scipy.linalg as spl
import sys
from pathlib import Path

from math import sin, cos, sqrt, asin, atan2, pi

sys.path.append(str(Path(__file__).absolute().parent) + "/../array")
from xy_array import XYArray


class SensorParameters:
    """
    Parameters class for sensor
    """

    def __init__(self, lon_m=0.0, lat_m=0.0, min_m=0.5, max_m=40, reso_deg=2.0,
                 angle_std_scale=0.01, dist_std_rate=0.005, calibrator=None):
        """
        Constructor
        lon_m: longitudinal installation position on vehicle coordinate system[m]
        lat_m: lateral installation position on vehicle coordinate system[m]
        min_m: minimum sensing range[m]
        max_m: maximum sensing range[m]
        reso_deg: resolution of sensing angle[deg]
        angle_std_scale: scale of angle's standard deviation
        dist_std_rate: rate of distance's standard deviation
        calibrator: instance of calibrator module
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

        # 3x3 homogeneous transformation matrix of sensor position and pose
        self.first_sensor_pos = True
        self.prev_sensor_tf = np.zeros((3, 3)) # at previous time
        self.curr_sensor_tf = np.zeros((3, 3)) # at current time

        # 3x3 homogeneous transformation matrix of vehicle position and pose
        self.first_vehicle_pos = True
        self.prev_vehicle_tf = np.zeros((3, 3))
        self.curr_vehicle_tf = np.zeros((3, 3))

        self.calibrator = calibrator
    
    def calculate_global_pos(self, state):
        """
        Function to calculate sensor's installation position on global coordinate system
        state: vehicle's state object
        """

        pose = state.x_y_yaw()
        transformed_array = self.inst_pos_array.homogeneous_transformation(pose[0, 0], pose[1, 0], pose[2, 0])
        
        self.global_x_m = transformed_array.get_x_data()
        self.global_y_m = transformed_array.get_y_data()
    
    def calculate_sensor_odometry(self, state):
        pose = state.x_y_yaw()

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

    def _observation_model(self, state, vehicle_odom_tf):
        state_tf = self._hom_mat(state[0, 0], state[1, 0], state[2, 0])
        return np.linalg.inv(state_tf) @ vehicle_odom_tf @ state_tf

    def _predict_sigmas_observation(self, sigmas, vehicle_odom_tf):
        sigmas_num = sigmas.shape[1]
        obv_sigmas = np.zeros((self.DIM_NUM, sigmas_num))
        for i in range(sigmas_num):
            sigma_tf = self._observation_model(sigmas[:, i:i+1], vehicle_odom_tf)
            obv_sigmas[0, i:i+1] = sigma_tf[0, 2]
            obv_sigmas[1, i:i+1] = sigma_tf[1, 2]
            obv_sigmas[2, i:i+1] = asin(sigma_tf[1, 0])
        return obv_sigmas
    
    def _calculate_correlation_matrix(self, pred_state, pred_sigmas, pred_obv, pred_obv_sigmas):
        sigmas_num = pred_sigmas.shape[1]
        diff_state = pred_sigmas - pred_state[0:pred_sigmas.shape[0]]
        diff_obv = pred_obv_sigmas - pred_obv[0:pred_obv_sigmas.shape[0]]
        corr_mat = np.zeros((diff_state.shape[0], diff_obv.shape[0]))
        for i in range(sigmas_num):
            corr_mat = corr_mat + self.COV_WEIGHTS[0, i] * diff_state[:, i:i+1] @ diff_obv[:, i:i+1].T
        return corr_mat

    def estimate_extrinsic_params(self, vehicle_state):
        if self.calibrator:
            # sensor odometry between 2 steps
            sensor_odom_tf = np.linalg.inv(self.prev_sensor_tf) @ self.curr_sensor_tf

            # vehicle odometry between 2 steps
            pose = vehicle_state.x_y_yaw()
            if self.first_vehicle_pos:
                self.prev_vehicle_tf = self._hom_mat(pose[0, 0], pose[1, 0], pose[2, 0])
                self.curr_vehicle_tf = self.prev_vehicle_tf
                self.first_vehicle_pos = False
            else:
                self.prev_vehicle_tf = self.curr_vehicle_tf
                self.curr_vehicle_tf = self._hom_mat(pose[0, 0], pose[1, 0], pose[2, 0])
            vehicle_odom_tf = np.linalg.inv(self.prev_vehicle_tf) @ self.curr_vehicle_tf

            self.calibrator.calibrate_extrinsic_params(sensor_odom_tf, vehicle_odom_tf)

        
        obv_vec = np.array([[sensor_odom_tf[0, 2]],
                            [sensor_odom_tf[1, 2]],
                            [asin(sensor_odom_tf[1, 0])]])

        # predict observation
        sigmas = self._generate_sigme_points(pred_state, pred_cov)
        pred_obv_sigmas = self._predict_sigmas_observation(sigmas, vehicle_odom_tf)
        pred_obv = (self.STATE_WEIGHTS @ pred_obv_sigmas.T).T
        pred_obv_cov = self._predict_covariance(pred_obv, pred_obv_sigmas, self.OBV_NOISE)
        
        # update state
        corr_mat = self._calculate_correlation_matrix(pred_state, pred_sigmas, pred_obv, pred_obv_sigmas)
        kalman_gain = corr_mat @ np.linalg.inv(pred_obv_cov)
        innovation = obv_vec - pred_obv
        upd_state = pred_state + kalman_gain @ innovation
        upd_cov = pred_cov - kalman_gain @ pred_obv_cov @ kalman_gain.T

        self.state = upd_state
        self.cov = upd_cov

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
    
    def draw_pos(self, axes, elems, state):
        """
        Function to draw sensor's installation position on vehicle
        axes: axes object of figure
        elems: list of plot object
        """

        self.calculate_global_pos(state)
        pos_plot, = axes.plot(self.global_x_m, self.global_y_m, marker='.', color='b')
        elems.append(pos_plot)

        # draw global position plot
        pose = state.x_y_yaw()
        global_tf = self._hom_mat(pose[0, 0], pose[1, 0], pose[2, 0])
        state_tf = self._hom_mat(self.state[0, 0], self.state[1, 0], self.state[2, 0])
        global_state_tf = global_tf @ state_tf
        state_plot, = axes.plot(global_state_tf[0, 2], global_state_tf[1, 2], marker='*', color='r')
        elems.append(state_plot)

        # draw calibration result text
        elems.append(axes.text(global_state_tf[0, 2], global_state_tf[1, 2] + 4,
                               "Sensor Lon Est:{0:.2f}/True:{1:.2f}[m]".format(self.state[0, 0], self.INST_LON_M),
                               fontsize=12))
        elems.append(axes.text(global_state_tf[0, 2], global_state_tf[1, 2] + 3.5,
                               "Sensor Lat Est:{0:.2f}/True:{1:.2f}[m]".format(self.state[1,0], self.INST_LAT_M),
                               fontsize=12))
        elems.append(axes.text(global_state_tf[0, 2], global_state_tf[1, 2] + 3.0,
                               "Sensor Yaw Est:{0:.2f}/True:{1:.2f}[deg]".format(np.rad2deg(self.state[2,0]), 0.0),
                               fontsize=12))

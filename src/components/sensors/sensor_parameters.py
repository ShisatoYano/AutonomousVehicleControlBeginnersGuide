"""
sensor_parameters.py

Author: Shisato Yano
"""

import numpy as np
import sys
from pathlib import Path

from math import sin, cos

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

        self.est_inst_array = np.zeros((3, 1))
        self.prev_sensor_tf = np.zeros((3, 3))
        self.curr_sensor_tf = np.zeros((3, 3))
        self.prev_vehicle_tf = np.zeros((3, 3))
        self.curr_vehicle_tf = np.zeros((3, 3))
        self.first_sensor_pos = True
        self.first_vehicle_pos = True
    
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

    def estimate_extrinsic_params(self, state):
        # current vehicle pose
        pose = state.x_y_yaw()
        # sensor odometry between 2 steps
        sensor_odom_glb_tf = np.linalg.inv(self.prev_sensor_tf) @ self.curr_sensor_tf
        sensor_odom_lcl_tf = np.linalg.inv(self._hom_mat(0.0, 0.0, pose[2, 0])) @ sensor_odom_glb_tf

        # vehicle odometry between 2 steps
        if self.first_vehicle_pos:
            self.prev_vehicle_tf = self._hom_mat(pose[0, 0], pose[1, 0], pose[2, 0])
            self.curr_vehicle_tf = self.prev_vehicle_tf
            self.first_vehicle_pos = False
        else:
            self.prev_vehicle_tf = self.curr_vehicle_tf
            self.curr_vehicle_tf = self._hom_mat(pose[0, 0], pose[1, 0], pose[2, 0])
            vehicle_odom_glb_tf = np.linalg.inv(self.prev_vehicle_tf) @ self.curr_vehicle_tf

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

"""
sensor_parameters.py

Author: Shisato Yano
"""

import numpy as np
import sys
from pathlib import Path

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
    
    def calculate_global_pos(self, state):
        """
        Function to calculate sensor's installation position on global coordinate system
        state: vehicle's state object
        """

        pose = state.x_y_yaw()
        transformed_array = self.inst_pos_array.homogeneous_transformation(pose[0, 0], pose[1, 0], pose[2, 0])
        self.global_x_m = transformed_array.get_x_data()
        self.global_y_m = transformed_array.get_y_data()
    
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

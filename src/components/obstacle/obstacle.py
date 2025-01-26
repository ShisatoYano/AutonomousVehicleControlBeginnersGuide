"""
obstacle.py

Author: Shisato Yano
"""

import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../array")
from xy_array import XYArray


class Obstacle:
    """
    Obstacle's data and logic class
    """

    def __init__(self, state, accel_mps2=0.0, yaw_rate_rps=0.0,
                 length_m=2.0, width_m=2.0):
        """
        Constructor
        state: Obstacle's state object
        accel_mps2: acceleration input to move[m/s2]
        yaw_rate_rps: yaw rate input to move[rad/s]
        length_m: Obstacle's longitudinal size[m]
        width_m: Obstacle's half of lateral size[m]
        """

        self.state = state
        self.accel_mps2 = accel_mps2
        self.yaw_rate_rps = yaw_rate_rps
        self.length_m = length_m
        self.width_m = width_m

        contour = np.array([[length_m, -length_m, -length_m, length_m, length_m],
                            [width_m, width_m, -width_m, -width_m, width_m]])

        self.array = XYArray(contour)

    def update(self, time_s):
        """
        Function to update obstacle's state
        time_s: Time interval value[sec]
        """
        
        self.state.update(self.accel_mps2, self.yaw_rate_rps, time_s)
    
    def draw(self, axes, elems):
        """
        Function to draw obstacle
        axes: Axes object of figure
        elems: List of plot objects
        """
        
        x_m = self.state.get_x_m()
        y_m = self.state.get_y_m()
        yaw_rad = self.state.get_yaw_rad()

        transformed_array = self.array.homogeneous_transformation(x_m, y_m, yaw_rad)
        obstacle_plot, = axes.plot(transformed_array.get_x_data(), 
                                   transformed_array.get_y_data(), 
                                   lw=1.0, color='k', ls='-')
        elems.append(obstacle_plot)
    
    def vertex_xy(self):
        """
        Function to get obstacle's vertex point coordinates
        """
        
        x_m = self.state.get_x_m()
        y_m = self.state.get_y_m()
        yaw_rad = self.state.get_yaw_rad()

        transformed_array = self.array.homogeneous_transformation(x_m, y_m, yaw_rad)

        return transformed_array.get_x_data(), transformed_array.get_y_data()

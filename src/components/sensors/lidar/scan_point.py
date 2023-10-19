"""
scan_point.py

Author: Shisato Yano
"""

import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../../array")
from xy_array import XYArray


class ScanPoint:
    """
    Scan point of sensor class includes each sensing data
    """
    
    def __init__(self, distance_m, angle_rad, x_m, y_m):
        """
        Constructor
        distance_m: sensed distance data[m]
        angle_rad: sensed angle data[rad]
        x_m: sensed point's x coordinate data[m]
        y_m: sensed point's y coordinate data[m]
        """
        
        self.distance_m = distance_m
        self.angle_rad = angle_rad
        self.point_array = XYArray(np.array([[x_m], [y_m]]))
    
    def get_dimension(self):
        """
        Return point's x-y array data's dimension value
        """

        return self.point_array.get_dimension()
    
    def get_distance_m(self):
        """
        Return point's distance data[m]
        """
        
        return self.distance_m
    
    def get_point_array(self):
        """
        Return point's x-y array data
        Type is ndarray object
        """

        return self.point_array.get_data()
    
    def draw(self, axes, x_m, y_m, angle_rad, elems):
        """
        Function to draw scan point's x-y coordinate data
        axes: Axes object of figure
        x_m: Vehicle's position x[m]
        y_m: Vehicle's position y[m]
        angle_rad: Vehicle's yaw angle[rad]
        elems: List of plot objects
        """

        transformed_array = self.point_array.homogeneous_transformation(x_m, y_m, angle_rad)
        point_plot, = axes.plot(transformed_array.get_x_data(), 
                                transformed_array.get_y_data(), 
                                marker='.', color='b')
        elems.append(point_plot)

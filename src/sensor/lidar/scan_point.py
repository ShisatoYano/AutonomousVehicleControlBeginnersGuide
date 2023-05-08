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
    def __init__(self, distance_m, angle_rad, x_m, y_m):
        self.distance_m = distance_m
        self.angle_rad = angle_rad
        self.point_array = XYArray(np.array([[x_m], [y_m]]))
    
    def draw(self, axes, x_m, y_m, angle_rad, elems):
        transformed_array = self.point_array.homogeneous_transformation(x_m, y_m, angle_rad)
        point_plot, = axes.plot(transformed_array.get_x_data(), 
                                transformed_array.get_y_data(), 
                                marker='.', color='b')
        elems.append(point_plot)

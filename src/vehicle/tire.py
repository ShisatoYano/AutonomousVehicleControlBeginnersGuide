"""
tire.py

Author: Shisato Yano
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../common")
from transformation import Transformation


class Tire:
    """
    Vehicle Tire class
    """

    def __init__(self, spec, x_m, y_m):
        """
        Constructor
        spec: vehicle specification object
        x_m: offset x[m] on vehicle
        y_m: offset y[m] on vehicle
        """
        
        self.r_m = spec.tire_r_m
        self.w_m = spec.tire_w_m
        self.x_m = x_m
        self.y_m = y_m
        self.color = spec.color
        self.line_w = spec.line_w
        self.line_type = spec.line_type

        self.points = np.array([
            [self.r_m, -self.r_m, -self.r_m, self.r_m, self.r_m],
            [self.w_m, self.w_m, -self.w_m, -self.w_m, self.w_m]
        ])

        self.points[0, :]
        self.points[1, :]
    
    def draw(self, axes, pose, steer_rad):
        base_steer = np.array([[self.x_m], [self.y_m], [steer_rad]])
        transformed_points = Transformation.homogeneous_transformation(self.points, base_steer)
        transformed_points = Transformation.homogeneous_transformation(transformed_points, pose)
        return axes.plot(transformed_points[0, :], 
                         transformed_points[1, :], 
                         lw=self.line_w, 
                         color=self.color, 
                         ls=self.line_type)

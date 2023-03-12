"""
axle.py

Author: Shisato Yano
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../common")
from transformation import Transformation


class Axle:
    """
    Vehicle Axle class
    """

    def __init__(self, spec, x_m, y_m):
        """
        Constructor
        spec: vehicle specification object
        x_m: offset x[m] on vehicle
        y_m: offset y[m] on vehicle
        """

        self.x_m = x_m
        self.y_m = y_m
        self.color = spec.color
        self.line_w = spec.line_w
        self.line_type = spec.line_type

        self.points = np.array([
            [0.0, 0.0],
            [self.y_m, -self.y_m]
        ])
    
    def draw(self, axes, pose):
        translated_points = Transformation.translation(self.points, self.x_m, 0.0)
        transformed_points = Transformation.homogeneous_transformation(translated_points, pose)
        return axes.plot(transformed_points[0, :], 
                         transformed_points[1, :], 
                         lw=self.line_w, 
                         color=self.color, 
                         ls=self.line_type)

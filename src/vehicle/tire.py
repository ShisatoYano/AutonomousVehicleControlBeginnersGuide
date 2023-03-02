"""
tire.py

Author: Shisato Yano
"""

import numpy as np
import matplotlib.pyplot as plt


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

        self.points[0, :] += self.x_m
        self.points[1, :] += self.y_m
    
    def draw_object(self, axes):
        return axes.plot(self.points[0, :], self.points[1, :], lw=self.line_w, color=self.color, ls=self.line_type)

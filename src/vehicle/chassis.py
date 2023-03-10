"""
chassis.py

Author: Shisato Yano
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../common")
from transformation import Transformation


class Chassis:
    """
    Vehicle Chassis class
    """

    def __init__(self, spec):
        """
        Constructor
        spec: vehicle specification object
        """
        
        self.f_len_m = spec.f_len_m
        self.r_len_m = spec.r_len_m
        self.color = spec.color
        self.line_w = spec.line_w
        self.line_type = spec.line_type

        self.points = np.array([
            [spec.f_len_m, -spec.r_len_m],
            [0.0, 0.0]
        ])
    
    def draw(self, axes, pose):
        transformed_points = Transformation.homogeneous_transformation(self.points, pose)
        return axes.plot(transformed_points[0, :], 
                         transformed_points[1, :], 
                         lw=self.line_w, 
                         color=self.color, 
                         ls=self.line_type)

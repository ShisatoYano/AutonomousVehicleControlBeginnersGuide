"""
body.py

Author: Shisato Yano
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../common")
from transformation import Transformation


class Body:
    """
    Vehicle Body class
    """

    def __init__(self, array):
        """
        Constructor
        spec: vehicle specification object
        """

        self.array = array
        
        # self.f_len_m = spec.f_len_m
        # self.r_len_m = spec.r_len_m
        # self.tread_m = spec.tread_m
        # self.f_edge_m = spec.f_edge_m
        # self.r_edge_m = spec.r_edge_m
        # self.width_m = spec.width_m
        # self.color = spec.color
        # self.line_w = spec.line_w
        # self.line_type = spec.line_type

        # self.points = np.array([
        #     [self.f_edge_m, -self.r_edge_m, -self.r_edge_m, self.f_edge_m, self.f_edge_m],
        #     [self.width_m, self.width_m, -self.width_m, -self.width_m, self.width_m]
        # ])
    
    def update(self, pose):
        updated_array = self.array.homogeneous_transformation(pose[0, 0], pose[1, 0], pose[2, 0])
        return Body(updated_array)

    def draw(self, axes, spec, pose):
        transformed_array = self.array.homogeneous_transformation(pose[0, 0], pose[1, 0], pose[2, 0])
        return axes.plot(transformed_array[0, :], 
                         transformed_array[1, :], 
                         lw=spec.line_w, 
                         color=spec.color, 
                         ls=spec.line_type)

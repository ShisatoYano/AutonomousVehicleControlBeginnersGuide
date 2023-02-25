"""
body.py

Author: Shisato Yano
"""

import numpy as np
import matplotlib.pyplot as plt


class Body:
    """
    Vehicle Body class
    """

    def __init__(self, spec):
        """
        Constructor
        spec: vehicle specification object
        """
        
        self.f_len_m = spec.f_len_m - 0.25
        self.r_len_m = spec.r_len_m - 0.25
        self.tread_m = 0.25 * (1.0 + spec.f_len_m + spec.r_len_m)
        self.f_edge_m = self.f_len_m + 0.75
        self.r_edge_m = self.r_len_m + 0.75
        self.width_m = 1.0 * self.tread_m
        self.color = spec.color
        self.line_w = spec.line_w
        self.line_type = spec.line_type

        self.points = np.array([
            [self.f_edge_m, -self.r_edge_m, -self.r_edge_m, self.f_edge_m, self.f_edge_m],
            [self.width_m, self.width_m, -self.width_m, -self.width_m, self.width_m]
        ])

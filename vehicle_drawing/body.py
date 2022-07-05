"""
Vehicle body drawing program

Author: Shisato Yano
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
from os.path import dirname, abspath

parent_dir = dirname(dirname(abspath(__file__)))
if parent_dir not in sys.path: sys.path.append(parent_dir)
from common.transformation import rotate_translate_2d


class Body:
    def __init__(self, axes, front_length_m, rear_length_m):
        self.front_length = front_length_m
        self.rear_length = rear_length_m
        self.tread_half = 0.4 * (front_length_m + rear_length_m)
        self.front_edge = self.front_length + 3
        self.rear_edge = self.rear_length + 3
        self.width_half = 1.5 * self.tread_half

        self.points = np.array([
            [self.front_edge, -self.rear_edge, -self.rear_edge, self.front_edge, self.front_edge],
            [self.width_half, self.width_half, -self.width_half, -self.width_half, self.width_half]
        ])

        self.plot, = axes.plot(self.points[0, :], self.points[1, :], lw=1, color='k')
    
    def draw(self, x_m, y_m, yaw_angle_deg):
        transformed_points = rotate_translate_2d(self.points, x_m, y_m, np.deg2rad(yaw_angle_deg))
        self.plot.set_data(transformed_points[0, :], transformed_points[1, :])

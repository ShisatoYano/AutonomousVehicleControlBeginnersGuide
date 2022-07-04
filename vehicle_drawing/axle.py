"""
Vehicle axle drawing program

Author: Shisato Yano
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
from os.path import dirname, abspath

parent_dir = dirname(dirname(abspath(__file__)))
if parent_dir not in sys.path: sys.path.append(parent_dir)
from common.transformation import rotate_translate_2d


class Axle:
    def __init__(self, axes, offset_x_m, offset_y_m):
        self.offset_x = offset_x_m
        self.offset_y = offset_y_m

        self.points = np.array([
            [0.0, 0.0],
            [self.offset_y, -self.offset_y]
        ])

        self.plot, = axes.plot(self.points[0, :], self.points[1, :], lw=1, color='k')
    
    def draw(self, x_m, y_m, yaw_angle_deg, steer_angle_deg):
        transformed_points = rotate_translate_2d(self.points, self.offset_x, 0.0, steer_angle_deg)
        transformed_points = rotate_translate_2d(transformed_points, x_m, y_m, yaw_angle_deg)
        self.plot.set_data(transformed_points[0, :], transformed_points[1, :])

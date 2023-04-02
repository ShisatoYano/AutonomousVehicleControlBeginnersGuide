"""
pure_pursuit_controller.py

Author: Shisato Yano
"""

from math import sin
import numpy as np


class SinCurveCourse:
    def __init__(self, x_min, x_max, resolution, target_speed_kmph):
        self.x_array = np.arange(x_min, x_max, resolution)
        self.y_array = [sin(x / 5.0) * (x / 2.0) for x in self.x_array]
        self.speed_array = [(target_speed_kmph / 3.6) for _ in self.x_array]
    
    def draw(self, axes, elems):
        elems += axes.plot(self.x_array, self.y_array, linewidth=0, marker='.', color='r')

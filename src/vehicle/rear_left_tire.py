"""
rear_left_tire.py

Author: Shisato Yano
"""

import numpy as np
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../array")
from xy_array import XYArray


class RearLeftTire:
    """
    Vehicle Rear Left Tire class
    """

    def __init__(self, spec):
        """
        Constructor
        spec: object of VehicleSpecification class
        """
        
        self.spec = spec
        self.offset_x_m = -spec.r_len_m
        self.offset_y_m = spec.axle_half_m

        contour = np.array([[self.spec.tire_r_m, -self.spec.tire_r_m, -self.spec.tire_r_m, self.spec.tire_r_m, self.spec.tire_r_m],
                            [self.spec.tire_w_m, self.spec.tire_w_m, -self.spec.tire_w_m, -self.spec.tire_w_m, self.spec.tire_w_m]])
        self.array = XYArray(contour)
    
    def draw(self, axes, pose, elems):
        translated_array = self.array.homogeneous_transformation(self.offset_x_m, self.offset_y_m, 0.0)
        transformed_array = translated_array.homogeneous_transformation(pose[0, 0], pose[1, 0], pose[2, 0])
        data = transformed_array.get_data()
        elems += axes.plot(data[0, :], data[1, :], lw=self.spec.line_w, color=self.spec.color, ls=self.spec.line_type)

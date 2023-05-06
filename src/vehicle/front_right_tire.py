"""
front_right_tire.py

Author: Shisato Yano
"""

import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../array")
from xy_array import XYArray


class FrontRightTire:
    """
    Vehicle Front Right Tire class
    """

    def __init__(self, spec):
        """
        Constructor
        spec: object of VehicleSpecification class
        """
        
        self.spec = spec
        self.offset_x_m = spec.f_len_m
        self.offset_y_m = -spec.axle_half_m

        contour = np.array([[self.spec.tire_r_m, -self.spec.tire_r_m, -self.spec.tire_r_m, self.spec.tire_r_m, self.spec.tire_r_m],
                            [self.spec.tire_w_m, self.spec.tire_w_m, -self.spec.tire_w_m, -self.spec.tire_w_m, self.spec.tire_w_m]])
        self.array = XYArray(contour)
    
    def draw(self, axes, pose, steer_rad, elems):
        translated_array = self.array.homogeneous_transformation(self.offset_x_m, self.offset_y_m, steer_rad)
        transformed_array = translated_array.homogeneous_transformation(pose[0, 0], pose[1, 0], pose[2, 0])
        tire_plot, = axes.plot(transformed_array.get_x_data(), transformed_array.get_y_data(), 
                               lw=self.spec.line_w, color=self.spec.color, ls=self.spec.line_type)
        elems.append(tire_plot)

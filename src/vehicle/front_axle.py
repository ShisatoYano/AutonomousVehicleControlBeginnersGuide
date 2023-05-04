"""
front_axle.py

Author: Shisato Yano
"""

import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../array")
from xy_array import XYArray


class FrontAxle:
    """
    Vehicle Front Axle class
    """

    def __init__(self, spec):
        """
        Constructor
        spec: object of VehicleSpecification class
        """

        self.spec = spec
        self.offset_x_m = spec.f_len_m
        self.offset_y_m = spec.axle_half_m

        contour = np.array([[0.0, 0.0], 
                            [self.offset_y_m, -self.offset_y_m]])
        self.array = XYArray(contour)
    
    def draw(self, axes, pose, elems):
        translated_array = self.array.homogeneous_transformation(self.offset_x_m, 0.0, 0.0)
        transformed_array = translated_array.homogeneous_transformation(pose[0, 0], pose[1, 0], pose[2, 0])
        data = transformed_array.get_data()
        axle_plot, = axes.plot(data[0, :], data[1, :], lw=self.spec.line_w, color=self.spec.color, ls=self.spec.line_type)
        elems.append(axle_plot)

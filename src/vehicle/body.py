"""
body.py

Author: Shisato Yano
"""

import numpy as np
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../array")
from xy_array import XYArray


class Body:
    """
    Vehicle Body class
    """

    def __init__(self, spec):
        """
        Constructor
        spec: object of VehicleSpecification class
        """

        self.spec = spec

        contour = np.array([[self.spec.f_edge_m, -self.spec.r_edge_m, -self.spec.r_edge_m, self.spec.f_edge_m, self.spec.f_edge_m],
                            [self.spec.width_m, self.spec.width_m, -self.spec.width_m, -self.spec.width_m, self.spec.width_m]])
        self.array = XYArray(contour)

    def draw(self, axes, pose, elems):
        transformed_array = self.array.homogeneous_transformation(pose[0, 0], pose[1, 0], pose[2, 0])
        data = transformed_array.get_data()
        elems += axes.plot(data[0, :], data[1, :], lw=self.spec.line_w, color=self.spec.color, ls=self.spec.line_type)

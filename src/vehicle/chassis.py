"""
chassis.py

Author: Shisato Yano
"""

import numpy as np
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../array")
from xy_array import XYArray


class Chassis:
    """
    Vehicle Chassis class
    """

    def __init__(self, spec):
        """
        Constructor
        spec: object of VehicleSpecification class
        """
        
        self.spec = spec

        contour = np.array([[self.spec.f_len_m, -self.spec.r_len_m],
                            [0.0, 0.0]])
        self.array = XYArray(contour)
    
    def draw(self, axes, pose):
        transformed_array = self.array.homogeneous_transformation(pose[0, 0], pose[1, 0], pose[2, 0])
        data = transformed_array.get_data()
        return axes.plot(data[0, :], data[1, :], 
                         lw=self.spec.line_w, 
                         color=self.spec.color, 
                         ls=self.spec.line_type)

"""
chassis.py

Author: Shisato Yano
"""

import numpy as np
import numpy.typing as npt
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../array")
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
    
    def transformed_outline(self, pose: npt.NDArray[np.float64]) -> XYArray:
        """
        Function to get vehicle's chassis outline transformed by vehicle's pose
        pose: Vehicle's pose vector
        Return transformed XYArray object
        """

        return self.array.homogeneous_transformation(pose[0, 0], pose[1, 0], pose[2, 0])

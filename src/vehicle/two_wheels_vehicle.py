"""
two_wheels_vehicle.py

Author: Shisato Yano
"""

import matplotlib.pyplot as plt
import numpy as np
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../visualization")
from global_xy_visualizer import GlobalXYVisualizer
from vehicle_specification import VehicleSpecification
from body import Body


class TwoWheelsVehicle:
    """
    Two Wheels Vehicle model class
    """

    def __init__(self, pose, spec):
        """
        Constructor
        pose: vehicle's pose [x[m], y[m], yaw[rad]]
        spec: vehicle's specification object
        """
        
        self.pose = pose
        self.spec = spec
        self.body = Body(self.spec)

    
    def draw(self, axes):
        x_m, y_m, yaw_rad = self.pose
        self.body.draw(axes)


def main():
    vis = GlobalXYVisualizer()

    spec = VehicleSpecification()
    vehicle = TwoWheelsVehicle(np.array([[0.0], [0.0], [0.0]]), spec)
    
    vis.add_object(vehicle)
    vis.draw()


if __name__ == "__main__":
    main()

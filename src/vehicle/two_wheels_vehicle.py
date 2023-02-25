"""
two_wheels_vehicle.py

Author: Shisato Yano
"""

import matplotlib.pyplot as plt
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../visualization")
from global_xy_visualizer import GlobalXYVisualizer
from vehicle_specification import VehicleSpecification


class TwoWheelsVehicle:
    """
    Two Wheels Vehicle model class
    """

    def __init__(self, pose, spec):
        self.pose = pose
        self.spec = spec
    
    def draw(self, axes):
        x_m, y_m, yaw_rad = self.pose

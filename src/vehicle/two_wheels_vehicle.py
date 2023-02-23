"""
two_wheels_vehicle.py

Author: Shisato Yano
"""

import matplotlib.pyplot as plt
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../visualization")
from global_xy_visualizer import GlobalXYVisualizer


class TwoWheelsVehicle:
    """
    Two Wheels Vehicle model class
    """

    def __init__(self, a_pose, front_length_m=2.0, rear_length_m=0.0, 
                 tire_radius_m=0.3, tire_width_half_m=0.12, color='k', 
                 line_width=1.0, line_type='-'):
        self.o_pose = a_pose

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

    def __init__(self, pose, f_len_m=2.0, r_len_m=0.0, tire_r_m=0.3,
                 tire_w_m=0.12, color='k', line_w=1.0, line_type='-'):
        self.pose = pose

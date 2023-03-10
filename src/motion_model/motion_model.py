"""
motion_model.py

Author: Shisato Yano
"""

import numpy as np
from math import cos, sin


class MotionModel:
    """
    Motion model class
    """

    def __init__(self, spec):
        self.spec = spec
    
    def state_transition(self, pose, order, time_interval_s):
        yaw_rad = pose[2, 0]
        return pose + np.array([[cos(yaw_rad) * time_interval_s, 0.0],
                                [sin(yaw_rad) * time_interval_s, 0.0],
                                [0.0, time_interval_s]]) @ order

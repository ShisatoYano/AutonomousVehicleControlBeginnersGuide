"""
extended_kalman_filter_localizer.py

Author: Shisato Yano
"""

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../../state")
from state import State

class ExtendedKalmanFilterLocalizer:
    """
    Self localization by Extended Kalman Filter class
    """

    def __init__(self, state, color='r'):
        """
        Constructor
        state: Vehicle's state object
        color: Color of drawing error covariance ellipse
        """
        
        self.state = state
        self.DRAW_COLOR = color
    
    def update(self, accel_mps2, yaw_rate_rps, time_s, gnss):
        print(gnss)

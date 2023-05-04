"""
obstacle.py

Author: Shisato Yano
"""

import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../array")
from xy_array import XYArray


class Obstacle:
    def __init__(self, state, length_m=2.0, width_m=2.0):
        self.state = state
        
        contour = np.array([[length_m, -length_m, -length_m, length_m, length_m],
                            [width_m, width_m, -width_m, -width_m, width_m]])
        self.array = XYArray(contour)

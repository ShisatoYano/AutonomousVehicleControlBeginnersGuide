"""
gnss.py

Author: Shisato Yano
"""

import numpy as np

class Gnss:
    """
    Position data sensing simulation class with GNSS
    """

    def __init__(self, x_m=0.0, y_m=0.0, 
                 x_noise_std=0.5, y_noise_std=0.5,
                 color='g'):
        """
        Constructor
        x_m: Initial observed position, x[m]
        y_m: Initial observed position, y[m]
        x_noise_std: Standard deviation of position x noise
        y_noise_std: Standard deviation of position y noise
        color: Color of drawing observed position point
        """
        
        self.NOISE_VAR_MAT = np.diag([x_noise_std, y_noise_std]) ** 2
        self.DRAW_COLOR = color

        self.x_m = x_m
        self.y_m = y_m

        self.x_history = [self.x_m]
        self.y_history = [self.y_m]

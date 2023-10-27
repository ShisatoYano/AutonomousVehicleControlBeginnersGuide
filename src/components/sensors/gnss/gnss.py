"""
gnss.py

Author: Shisato Yano
"""

import numpy as np

class Gnss:
    """
    GNSS position data sensing simulation class
    """

    def __init__(self, x_noise_std=0.5, y_noise_std=0.5, color='g'):
        """
        Constructor
        x_noise_std: Standard deviation of position x noise
        y_noise_std: Standard deviation of position y noise
        color: Color of drawing observed position point
        """
        
        self.NOISE_VAR_MAT = np.diag([x_noise_std, y_noise_std]) ** 2
        self.DRAW_COLOR = color

        self.latest_observed_xy = None
        self.x_history = []
        self.y_history = []
    
    @staticmethod
    def observation_model(state):
        """
        Static function of observation model of vehicle state
        state: Vehicle's state (x, y, yaw, speed) object
        """
        
        H = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0]])
        
        x = np.array([[state.get_x_m()],
                      [state.get_y_m()],
                      [state.get_yaw_rad()],
                      [state.get_speed_mps()]])
        
        return H @ x
    
    def update(self, state):
        """
        Function to update GNSS observation data (x, y)
        state: Vehicle's state (x, y, yaw, speed) object
        """
        
        observed_noise = self.NOISE_VAR_MAT @ np.random.randn(2, 1)
        observed_xy = self.observation_model(state) + observed_noise
        
        self.latest_observed_xy = observed_xy
        self.x_history.append(self.latest_observed_xy[0])
        self.y_history.append(self.latest_observed_xy[1])
    
    def draw(self, axes, elems):
        """
        Function to draw GNSS observation position (x, y)
        axes: Axes object of figure
        elems: List of plot object
        """
        
        hist_plot, = axes.plot(self.x_history, self.y_history, linewidth=0, marker='.', color=self.DRAW_COLOR)
        elems.append(hist_plot)

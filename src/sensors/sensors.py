"""
sensors.py

Author: Shisato Yano
"""

class Sensors:
    """
    Each sensors logic class
    """
    
    def __init__(self, lidar=None, gnss=None):
        """
        Constructor
        lidar: lidar object
        gnss: gnss object
        """

        self.lidar = lidar
        self.gnss = gnss
    
    def install(self, state):
        """
        Function to calculate each sensor's installation position on vehicle
        state: vehicle's state object
        """

        if self.lidar: self.lidar.install(state)

    def update_data(self, state):
        """
        Function to update each sensor's data
        state: vehicle's state object
        """

        if self.lidar: self.lidar.update(state)
    
    def draw_data(self, axes, elems, state):
        """
        Function to draw each sensor's data
        axes: axes object of figure
        elems: list of plot object
        state: vehicle's state object
        """

        if self.lidar: self.lidar.draw(axes, elems, state)

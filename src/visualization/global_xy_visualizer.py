"""
global_xy_visualizer.py

Author: Shisato Yano
"""

import matplotlib.pyplot as plt


class GlobalXYVisualizer:
    """
    Visualization class for global 2D X-Y plot
    """
    
    def __init__(self):
        self.objects = [] # register objects here
    
    def add_object(self, aObject):
        """
        Method to register object
        """

        self.objects.append(aObject)

"""
rectangle.py

Author: Shisato Yano
"""

import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../../array")
from xy_array import XYArray

class Rectangle:
    """
    Rectangle shape class
    """

    def __init__(self):
        """
        Constructor
        """
        
        self.EDGE_NUM = 4
        self.VERTEX_NUM = self.EDGE_NUM + 1

        edge_param_list = [None] * self.EDGE_NUM
        self.a = edge_param_list
        self.b = edge_param_list
        self.c = edge_param_list

        vertex_list = [None] * self.VERTEX_NUM
        vertex_xs = vertex_list
        vertex_ys = vertex_list
        contour = np.array([vertex_xs,
                            vertex_ys])
        self.array = XYArray(contour)

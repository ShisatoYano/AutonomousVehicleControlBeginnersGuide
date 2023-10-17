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
    Consists of 4 edges, a*x + b*y = c
    """

    def __init__(self, a, b, c):
        """
        Constructor
        a: List of coefficient a in 4 edges
        b: List of coefficient b in 4 edges
        c: List of coefficient c in 4 edges
        """
        
        # edge parameters
        self.a = a # a_1-4
        self.b = b # b_1-4
        self.c = c # c_1-4

        # contour points
        vertex_num = len(self.a) + 1 # edge num of 4 + 1 to close rectangle
        self.contour_x = [None] * vertex_num
        self.contour_y = [None] * vertex_num
        self._calculate_contour()
        self.contour = XYArray(np.array([self.contour_x, 
                                         self.contour_y]))

    @staticmethod
    def _calculate_cross_point(a, b, c):
        x = (b[0] * c[1] - b[1] * c[0]) / (b[0] * a[1] - b[1] * a[0])
        y = (a[0] * c[1] - a[1] * c[0]) / (a[0] * b[1] - a[1] * b[0])
        return x, y
    
    def _calculate_contour(self):
        self.contour_x[0], self.contour_y[0] = self._calculate_cross_point(self.a[0:2], self.b[0:2], self.c[0:2])
        self.contour_x[1], self.contour_y[1] = self._calculate_cross_point(self.a[1:3], self.b[1:3], self.c[1:3])
        self.contour_x[2], self.contour_y[2] = self._calculate_cross_point(self.a[2:4], self.b[2:4], self.c[2:4])
        self.contour_x[3], self.contour_y[3] = self._calculate_cross_point([self.a[3], self.a[0]], [self.b[3], self.b[0]], [self.c[3], self.c[0]])
        self.contour_x[4], self.contour_y[4] = self.contour_x[0], self.contour_y[0]

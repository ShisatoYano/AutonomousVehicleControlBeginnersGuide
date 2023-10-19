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
        
        # center of point
        self.center_x = 0.0
        self.center_y = 0.0
        self._calculate_center()
        self.center_xy = XYArray(np.array([[self.center_x], [self.center_y]]))

    @staticmethod
    def _calculate_cross_point(a, b, c):
        """
        Private function to calculate cross point between two edges
        a: List of coefficient a in 2 edges
        b: List of coefficient b in 2 edges
        c: List of coefficient c in 2 edges
        Return x, y of cross point
        """
        
        x = (b[0] * c[1] - b[1] * c[0]) / (b[0] * a[1] - b[1] * a[0])
        y = (a[0] * c[1] - a[1] * c[0]) / (a[0] * b[1] - a[1] * b[0])
        return x, y
    
    def _calculate_contour(self):
        """
        Private function to calculate contour points of rectangle
        """
        
        self.contour_x[0], self.contour_y[0] = self._calculate_cross_point(self.a[0:2], self.b[0:2], self.c[0:2])
        self.contour_x[1], self.contour_y[1] = self._calculate_cross_point(self.a[1:3], self.b[1:3], self.c[1:3])
        self.contour_x[2], self.contour_y[2] = self._calculate_cross_point(self.a[2:4], self.b[2:4], self.c[2:4])
        self.contour_x[3], self.contour_y[3] = self._calculate_cross_point([self.a[3], self.a[0]], [self.b[3], self.b[0]], [self.c[3], self.c[0]])
        self.contour_x[4], self.contour_y[4] = self.contour_x[0], self.contour_y[0] # add to close rectangle drawing

    def _calculate_center(self):
        """
        Private function to calculate center point of rectangle
        """
        
        num = len(self.contour_x) - 1
        sum_x, sum_y = 0.0, 0.0
        for i in range(num):
            sum_x += self.contour_x[i]
            sum_y += self.contour_y[i]
        self.center_x = sum_x / num
        self.center_y = sum_y / num

    def draw(self, axes, elems, x_m, y_m, angle_rad):
        """
        Function to draw rectangle
        axes: Axes object of figure
        elems: List of plot objects
        x_m: Vehicle's position x[m]
        y_m: Vehicle's position y[m]
        angle_rad: Vehicle's yaw angle[rad]
        """
        
        # rectangle's contour points
        transformed_contour = self.contour.homogeneous_transformation(x_m, y_m, angle_rad)
        rectangle_plot, = axes.plot(transformed_contour.get_x_data(),
                                    transformed_contour.get_y_data(),
                                    color='g', ls='-')
        elems.append(rectangle_plot)

        # rectangle's center point
        transformed_center = self.center_xy.homogeneous_transformation(x_m, y_m, angle_rad)
        center_plot, = axes.plot(transformed_center.get_x_data(),
                                 transformed_center.get_y_data(),
                                 marker='.', color='g')
        elems.append(center_plot)

"""
cubic_spline.py

Author: Shisato Yano
"""

import numpy as np
import bisect


class CubicSpline:
    """
    1D Cubic Spline class
    Each section between two points are approximated as the following equation
    y_i = a_i + b_i * (x - x_i) + c_i * (x - x_i)^2 + d_i * (x - x_i)^3
    """

    def __init__(self, x_points, y_points):
        """
        Constructor
        x_points: List of x coordinate points. This must be stored in ascending order.
        y_points: List of y coordinate points.
        """
        
        h = np.diff(x_points)
        if np.any(h < 0):
            raise ValueError("X coordinate points must be stored in ascending order")
        
        self.a, self.b, self.c, self.d = [], [], [], []
        self.x_points = x_points
        self.y_points = y_points
        self.size_x_points = len(self.x_points)

        self._calculate_coefficient_a()
        self._calculate_coefficient_c(h)
        self._calculate_coefficient_b_d(h)
    
    def calculate_y(self, x):
        if x < self.x_points[0]: return None
        elif x > self.x_points[-1]: return None

        i_x = self._search_segment_index(x)
        dx = x - i_x
        y = self.a[i_x] + self.b[i_x] * dx + \
            self.c[i_x] * dx ** 2.0 + self.d[i_x] * dx ** 3.0
        
        return y

    def _search_segment_index(self, x):
        return bisect.bisect(self.x_points, x) - 1
    
    def _calculate_coefficient_a(self):
        self.a = [y_point for y_point in self.y_points]
    
    def _calculate_coefficient_c(self, h):
        A = self._calculate_matrix_A(h)
        B = self._calculate_matrix_B(h, self.a)
        self.c = np.linalg.solve(A, B)

    def _calculate_coefficient_b_d(self, h):
        for i in range(self.size_x_points - 1):
            d = (self.c[i + 1] - self.c[i]) / (3.0 * h[i])
            b = 1.0 / h[i] * (self.a[i + 1] - self.a[i]) \
                - h[i] / 3.0 * (2.0 * self.c[i] + self.c[i + 1])
            self.d.append(d)
            self.b.append(b)

    def _calculate_matrix_A(self, h):
        A = np.zeros((self.size_x_points, self.size_x_points))
        A[0, 0] = 1.0
        for i in range(self.size_x_points - 1):
            if i != (self.size_x_points - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]
        A[0, 1] = 0.0
        A[self.size_x_points - 1, self.size_x_points - 2] = 0.0
        A[self.size_x_points - 1, self.size_x_points - 1] = 1.0
        return A
    
    def _calculate_matrix_B(self, h, a):
        B = np.zeros(self.size_x_points)
        for i in range(self.size_x_points - 2):
            B[i + 1] = 3.0 * (a[i + 2] - a[i + 1]) / h[i + 1] \
                        - 3.0 * (a[i + 1] - a[i]) / h[i]
        return B

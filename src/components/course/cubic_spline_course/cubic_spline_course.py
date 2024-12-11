"""
cubic_spline_course.py

Author: Shisato Yano
"""

# import path setting
import sys
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"

sys.path.append(abs_dir_path + relative_path + "course/cubic_spline_course")

#import component modules
from cubic_spline_2d import CubicSpline2D


class CubicSplineCourse:
    """
    Course generation class by Cubic spline interpolation
    """

    def __init__(self, x_ref_points, y_ref_points, target_speed_kmph, resolution=0.1, color='r'):
        """
        Constructor
        x_ref_points: Array of reference points in x axis
        y_ref_points: Array of reference points in y axis
        target_speed_kmph: Target speed[km/h] driving on generated course
        resolution: Distance between 2 interpolated points
        color: Color of drawing course points
        """
        
        cubic_spline = CubicSpline2D(x_ref_points, y_ref_points)
        base_points = np.arange(0, cubic_spline.s[-1], resolution)

        self.x_array, self.y_array = [], []
        self.yaw_array, self.curvature_array = [], []
        for base_point in base_points:
            x, y = cubic_spline.calc_interpolated_xy(base_point)
            self.x_array.append(x)
            self.y_array.append(y)
            self.yaw_array.append(cubic_spline.calc_yaw_angle(base_point))
            self.curvature_array.append(cubic_spline.calc_curvature(base_point))
        
        self.speed_array = [(target_speed_kmph / 3.6) for _ in self.x_array]
        self.speed_array[-1] = 0.0

        self.color = color
    
    def search_nearest_point_index(self, state):
        """
        Function to search nearest point's index on course
        state: Vehicle's state object
        """

        vehicle_pos_x_m = state.get_x_m()
        vehicle_pos_y_m = state.get_y_m()

        diff_x_array = [vehicle_pos_x_m - point_x_m for point_x_m in self.x_array]
        diff_y_array = [vehicle_pos_y_m - point_y_m for point_y_m in self.y_array]
        diff_array = np.hypot(diff_x_array, diff_y_array)

        nearest_index = np.argmin(diff_array)
        
        return nearest_index

    def calculate_distance_from_point(self, state, point_index):
        """
        Function to calculate distance from a point on course
        state: Vehicle's state object
        point_index: index of point on course
        """
        
        diff_x_m = state.get_x_m() - self.x_array[point_index]
        diff_y_m = state.get_y_m() - self.y_array[point_index]
        return np.hypot(diff_x_m, diff_y_m)

    def draw(self, axes, elems):
        """
        Function to draw points on course
        axes: Axes object of figure
        elems: List of plot objects
        """
        
        course_plot, = axes.plot(self.x_array, self.y_array, linewidth=0, marker='.', color=self.color, label="Course")
        elems.append(course_plot)

"""
cubic_spline_course.py

Author: Shisato Yano
"""

# import path setting
import sys
from pathlib import Path
from math import sin, cos, atan2
import matplotlib.pyplot as plt
import numpy as np

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"

sys.path.append(abs_dir_path + relative_path + "course/cubic_spline_course")
sys.path.append(abs_dir_path + relative_path + "common")

#import component modules
from cubic_spline_2d import CubicSpline2D
from angle_lib import pi_to_pi


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
        
        self.target_speed_mps = target_speed_kmph / 3.6
        self.speed_array = [self.target_speed_mps for _ in self.x_array]

        prev_x, prev_y = 0.0, 0.0
        diff_xy = 0.0
        self.length_m = 0.0
        for i, (x, y) in enumerate(zip(self.x_array, self.y_array)):
            if i > 0:
                diff_xy = np.hypot(x - prev_x, y - prev_y)
                self.length_m += diff_xy
            prev_x = x
            prev_y = y

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

    def length(self):
        """
        Function to get length of course
        """
        
        return self.length_m

    def calculate_speed_difference_mps(self, state, point_index):
        """
        Function to calculate difference between current speed and target speed
        state: Vehicle's state object
        point_index: index of point on course
        """
        
        return self.speed_array[point_index] - state.get_speed_mps()

    def calculate_angle_difference_rad(self, state, point_index):
        """
        Function to calculate difference between current yaw angle and target point
        state: Vehicle's state object
        point_index: index of point on course
        """
        
        diff_x_m = self.x_array[point_index] - state.get_x_m()
        diff_y_m = self.y_array[point_index] - state.get_y_m()
        return atan2(diff_y_m, diff_x_m) - state.get_yaw_rad()

    def calculate_lonlat_error(self, state, point_index):
        """
        Function to calculate longitudinal/lateral/yaw angle error against course
        state: Vehicle's state object
        point_index: index of point on course
        """
        
        error_x_m = state.get_x_m() - self.x_array[point_index]
        error_y_m = state.get_y_m() - self.y_array[point_index]

        current_yaw_rad = state.get_yaw_rad()
        error_yaw_rad = pi_to_pi(current_yaw_rad - self.yaw_array[point_index])

        error_lon_m = cos(current_yaw_rad) * error_x_m + sin(current_yaw_rad) * error_y_m
        error_lat_m = -sin(current_yaw_rad) * error_x_m + cos(current_yaw_rad) * error_y_m

        return error_lon_m, error_lat_m, error_yaw_rad

    def point_x_m(self, point_index):
        """
        Function to get x coordinate[m] of point on course
        point_index: index of point on course
        """
        
        return self.x_array[point_index]
    
    def point_y_m(self, point_index):
        """
        Function to get y coordinate[m] of point on course
        point_index: index of point on course
        """

        return self.y_array[point_index]
    
    def point_yaw_rad(self, point_index):
        """
        Function to get yaw angle[rad] of point on course
        point_index: index of point on course
        """

        return self.yaw_array[point_index]
    
    def point_speed_mps(self, point_index):
        """
        Function to get speed[m/s] of point on course
        point_index: index of point on course
        """

        return self.speed_array[point_index]

    def max_speed_mps(self):
        """
        Function to get maximum speed[m/s] on course
        """
        
        return self.target_speed_mps

    def point_curvature(self, point_index):
        """
        Function to get curvature of point on course
        point_index: index of point on course
        """

        return self.curvature_array[point_index]

    def draw(self, axes, elems):
        """
        Function to draw points on course
        axes: Axes object of figure
        elems: List of plot objects
        """
        
        course_plot, = axes.plot(self.x_array, self.y_array, linewidth=0, marker='.', color=self.color, label="Course")
        elems.append(course_plot)

"""
sin_curve_course.py

Author: Shisato Yano
"""

from math import sin, atan2
import numpy as np


class SinCurveCourse:
    """
    Course generation class has sin curve shape
    """
    
    def __init__(self, x_min, x_max, resolution, target_speed_kmph):
        """
        Constructor
        x_min: Minimum value of x coordinate
        x_max: Maximum value of y coordinate
        resolution: Resolution[m] of x coordinates list
        target_speed_kmph: Target speed[km/h] driving on generated course
        """

        self.x_array = np.arange(x_min, x_max, resolution)
        self.y_array = [sin(x / 5.0) * (x / 2.0) for x in self.x_array]
        self.speed_array = [(target_speed_kmph / 3.6) for _ in self.x_array]
        self.speed_array[-1] = 0.0
    
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
    
    def target_speed_mps(self, point_index):
        """
        Function to get target speed[m/s] at a point on course
        point_index: index of point on course
        """

        return self.speed_array[point_index]
    
    def length(self):
        """
        Function to get length of course
        """
        
        return len(self.x_array)

    def draw(self, axes, elems):
        """
        Function to draw points on course
        axes: Axes object of figure
        elems: List of plot objects
        """
        
        course_plot, = axes.plot(self.x_array, self.y_array, linewidth=0, marker='.', color='r', label="Course")
        elems.append(course_plot)

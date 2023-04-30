"""
sin_curve_course.py

Author: Shisato Yano
"""

from math import sin, atan2
import numpy as np


class SinCurveCourse:
    def __init__(self, x_min, x_max, resolution, target_speed_kmph):
        self.x_array = np.arange(x_min, x_max, resolution)
        self.y_array = [sin(x / 5.0) * (x / 2.0) for x in self.x_array]
        self.speed_array = [(target_speed_kmph / 3.6) for _ in self.x_array]
        self.speed_array[-1] = 0.0
    
    def search_nearest_point_index(self, state):
        vehicle_pos_x_m = state.get_x_m()
        vehicle_pos_y_m = state.get_y_m()

        diff_x_array = [vehicle_pos_x_m - point_x_m for point_x_m in self.x_array]
        diff_y_array = [vehicle_pos_y_m - point_y_m for point_y_m in self.y_array]
        diff_array = np.hypot(diff_x_array, diff_y_array)

        nearest_index = np.argmin(diff_array)
        
        return nearest_index
    
    def calculate_distance_from_point(self, state, point_index):
        diff_x_m = state.get_x_m() - self.x_array[point_index]
        diff_y_m = state.get_y_m() - self.y_array[point_index]
        return np.hypot(diff_x_m, diff_y_m)
    
    def calculate_speed_difference_mps(self, state, point_index):
        return self.speed_array[point_index] - state.get_speed_mps()
    
    def calculate_angle_difference_rad(self, state, point_index):
        diff_x_m = self.x_array[point_index] - state.get_x_m()
        diff_y_m = self.y_array[point_index] - state.get_y_m()
        return atan2(diff_y_m, diff_x_m) - state.get_yaw_rad()
    
    def point_x_m(self, point_index):
        return self.x_array[point_index]
    
    def point_y_m(self, point_index):
        return self.y_array[point_index]
    
    def target_speed_mps(self, point_index):
        return self.speed_array[point_index]
    
    def length(self):
        return len(self.x_array)

    def draw(self, axes, elems):
        course_plot, = axes.plot(self.x_array, self.y_array, linewidth=0, marker='.', color='r', label="Course")
        elems.append(course_plot)

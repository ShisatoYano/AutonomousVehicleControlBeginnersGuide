"""
pure_pursuit_controller.py

Author: Shisato Yano
"""

from math import sin, tan, atan2
import numpy as np
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../visualization")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../state")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../vehicle")
from global_xy_visualizer import GlobalXYVisualizer
from vehicle_specification import VehicleSpecification
from state import State
from state_history import StateHistory
from four_wheels_vehicle import FourWheelsVehicle


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
        elems += axes.plot(self.x_array, self.y_array, linewidth=0, marker='.', color='r', label="Course")


class PurePursuitController:
    def __init__(self, spec, course=None):
        self.MIN_LOOK_AHEAD_DISTANCE_M = 2.0
        self.LOOK_FORWARD_GAIN = 0.3
        self.SPEED_PROPORTIONAL_GAIN = 1.0
        self.WHEEL_BASE_M = spec.wheel_base_m

        self.course = course
        self.look_ahead_distance_m = self.MIN_LOOK_AHEAD_DISTANCE_M
        self.target_course_index = 0
        self.target_accel_mps2 = 0.0
        self.target_steer_rad = 0.0
        self.target_yaw_rate_rps = 0.0
    
    def update(self, state):
        if not self.course: return

        # calculate look ahead distance
        self.look_ahead_distance_m = self.LOOK_FORWARD_GAIN * state.get_speed_mps() + self.MIN_LOOK_AHEAD_DISTANCE_M

        # calculate target course index
        nearest_index = self.course.search_nearest_point_index(state)
        while self.look_ahead_distance_m > self.course.calculate_distance_from_point(state, nearest_index):
            if nearest_index + 1 >= self.course.length(): break
            nearest_index += 1
        self.target_course_index = nearest_index

        # calculate target acceleration
        diff_speed_mps = self.course.calculate_speed_difference_mps(state, self.target_course_index)
        self.target_accel_mps2 = self.SPEED_PROPORTIONAL_GAIN * diff_speed_mps

        # calculate difference angle against course
        diff_angle_rad = self.course.calculate_angle_difference_rad(state, self.target_course_index)

        # calculate target steering angle to course
        self.target_steer_rad = atan2((2 * self.WHEEL_BASE_M * sin(diff_angle_rad)), self.look_ahead_distance_m)

        # calculate target yaw rate to course
        self.target_yaw_rate_rps = state.get_speed_mps() * tan(self.target_steer_rad) / self.WHEEL_BASE_M
    
    def get_target_accel_mps2(self):
        return self.target_accel_mps2
    
    def get_target_steer_rad(self):
        return self.target_steer_rad

    def get_target_yaw_rate_rps(self):
        return self.target_yaw_rate_rps
    
    def draw(self, axes, elems):
        elems += axes.plot(self.course.point_x_m(self.target_course_index), 
                           self.course.point_y_m(self.target_course_index), 
                           marker='o', color='g', label="Target Point")


def main():
    vis = GlobalXYVisualizer(x_min=-5, x_max=55, y_min=-20, y_max=25, time_span_s=30)

    course = SinCurveCourse(0, 50, 0.5, 20)
    vis.add_object(course)

    spec = VehicleSpecification()
    state = State(0.0, 0.0, 0.0, 0.0)
    history = StateHistory([state.get_x_m()], [state.get_y_m()], spec.color)
    
    pure_pursuit = PurePursuitController(spec, course)

    vehicle = FourWheelsVehicle(state, history, spec, controller=pure_pursuit)
    vis.add_object(vehicle)

    vis.draw()


if __name__ == "__main__":
    main()

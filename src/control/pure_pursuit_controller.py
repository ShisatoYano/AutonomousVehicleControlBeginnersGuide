"""
pure_pursuit_controller.py

Author: Shisato Yano
"""

from math import sin
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
    
    def draw(self, axes, elems):
        elems += axes.plot(self.x_array, self.y_array, linewidth=0, marker='.', color='r')


class PurePursuitController:
    def __init__(self, spec, course=None):
        self.MIN_LOOK_AHEAD_DISTANCE_M = 2.0
        self.LOOK_FORWARD_GAIN = 0.1
        self.SPEED_PROPORTIONAL_GAIN = 1.0
        self.WHEEL_BASE_M = spec.wheel_base_m

        self.course = course
    
    def accel_steer_input(self, state):
        if not self.course: return 0.0, 0.0
        
        return 0.0, 0.0


def main():
    vis = GlobalXYVisualizer(x_min=-5, x_max=55, y_min=-20, y_max=25, time_span_s=10)

    course = SinCurveCourse(0, 50, 0.5, 20)
    vis.add_object(course)

    spec = VehicleSpecification()
    state = State(0.0, 0.0, 0.0, 0.0)
    history = StateHistory([state.get_x_m()], [state.get_y_m()])
    vehicle = FourWheelsVehicle(state, history, spec)
    vis.add_object(vehicle)

    vis.draw()


if __name__ == "__main__":
    main()

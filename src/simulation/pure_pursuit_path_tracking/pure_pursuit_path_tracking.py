"""
pure_pursuit_path_tracking.py

Author: Shisato Yano
"""

import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../visualization")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../state")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../vehicle")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../course/sin_curve_course")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../control/pure_pursuit")
from global_xy_visualizer import GlobalXYVisualizer
from vehicle_specification import VehicleSpecification
from state import State
from state_history import StateHistory
from four_wheels_vehicle import FourWheelsVehicle
from sin_curve_course import SinCurveCourse
from pure_pursuit_controller import PurePursuitController


def main():
    vis = GlobalXYVisualizer(x_min=-5, x_max=55, y_min=-20, y_max=25, time_span_s=25)

    course = SinCurveCourse(0, 50, 0.5, 20)
    vis.add_object(course)

    spec = VehicleSpecification()
    state = State(0.0, 0.0, 0.0, 0.0)
    history = StateHistory([state.get_x_m()], [state.get_y_m()], spec.color)
    
    pure_pursuit = PurePursuitController(spec, course)

    vehicle = FourWheelsVehicle(state, history, spec, controller=pure_pursuit,
                                draw_area_width=20.0)
    vis.add_object(vehicle)

    vis.draw()


if __name__ == "__main__":
    main()

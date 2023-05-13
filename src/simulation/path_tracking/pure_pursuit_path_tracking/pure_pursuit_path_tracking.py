"""
pure_pursuit_path_tracking.py

Author: Shisato Yano
"""

import sys
from pathlib import Path

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../"

sys.path.append(abs_dir_path + relative_path + "visualization")
sys.path.append(abs_dir_path + relative_path + "state")
sys.path.append(abs_dir_path + relative_path + "vehicle")
sys.path.append(abs_dir_path + relative_path + "course/sin_curve_course")
sys.path.append(abs_dir_path + relative_path + "control/pure_pursuit")

from global_xy_visualizer import GlobalXYVisualizer
from min_max import MinMax
from time_parameters import TimeParameters
from vehicle_specification import VehicleSpecification
from state import State
from four_wheels_vehicle import FourWheelsVehicle
from sin_curve_course import SinCurveCourse
from pure_pursuit_controller import PurePursuitController


show_plot = True


def main():
    x_lim, y_lim = MinMax(-5, 55), MinMax(-20, 25)
    vis = GlobalXYVisualizer(x_lim, y_lim, TimeParameters(span_sec=25))

    course = SinCurveCourse(0, 50, 0.5, 20)
    vis.add_object(course)

    spec = VehicleSpecification(area_size=20.0)
    state = State(color=spec.color)
    
    pure_pursuit = PurePursuitController(spec, course)

    vehicle = FourWheelsVehicle(state, spec, controller=pure_pursuit)
    vis.add_object(vehicle)

    if not show_plot: vis.not_show_plot()

    vis.draw()


if __name__ == "__main__":
    main()

"""
mppi_path_tracking.py

Path tracking simulation using MPPI (Model Predictive Path Integral) controller.
Visualizes all evaluated trajectories and the chosen trajectory.
"""

import sys
from pathlib import Path

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"

sys.path.append(abs_dir_path + relative_path + "visualization")
sys.path.append(abs_dir_path + relative_path + "state")
sys.path.append(abs_dir_path + relative_path + "vehicle")
sys.path.append(abs_dir_path + relative_path + "course/cubic_spline_course")
sys.path.append(abs_dir_path + relative_path + "control/mppi")

from global_xy_visualizer import GlobalXYVisualizer
from min_max import MinMax
from time_parameters import TimeParameters
from vehicle_specification import VehicleSpecification
from state import State
from four_wheels_vehicle import FourWheelsVehicle
from cubic_spline_course import CubicSplineCourse
from mppi_controller import MppiController

show_plot = True


def main():
    x_lim, y_lim = MinMax(-5, 55), MinMax(-20, 25)
    gif_path = str(Path(__file__).absolute().parent / "mppi_path_tracking.gif")
    vis = GlobalXYVisualizer(
        x_lim, y_lim, TimeParameters(span_sec=25), gif_name=gif_path
    )

    course = CubicSplineCourse([0.0, 10.0, 25, 40, 50], [0.0, 4, -12, 20, -13], 20)
    vis.add_object(course)

    spec = VehicleSpecification(area_size=20.0)
    state = State(color=spec.color)

    mppi = MppiController(
        spec,
        course,
        color="g",
        delta_t=0.1,
        horizon_step_T=22,
        number_of_samples_K=200,
        param_exploration=0.05,
        param_lambda=50.0,
        param_alpha=0.98,
        sigma_steer=0.2,
        sigma_accel=0.8,
        max_steer_abs=0.523,
        max_accel_abs=2.0,
        stage_cost_weight=[50.0, 50.0, 1.0, 20.0],
        terminal_cost_weight=[50.0, 50.0, 1.0, 20.0],
        moving_average_window=5,
        visualize_optimal_traj=True,
        visualize_sampled_trajs=True,
    )

    vehicle = FourWheelsVehicle(state, spec, controller=mppi)
    vis.add_object(vehicle)

    if not show_plot:
        vis.not_show_plot()
    vis.draw()


if __name__ == "__main__":
    main()

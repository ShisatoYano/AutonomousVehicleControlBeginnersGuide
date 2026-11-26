"""
dynamic_window_approach.py

Title: Dynamic Window Approach
Description: Picks the speed and yaw rate reachable in one step whose short predicted trajectory heads to the goal and keeps away from obstacles
Author: Yuvraj Gupta
"""

# import path setting
import sys
from pathlib import Path
import numpy as np

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"
relative_simulations = "/../../../simulations/"

sys.path.append(abs_dir_path + relative_path + "visualization")
sys.path.append(abs_dir_path + relative_path + "state")
sys.path.append(abs_dir_path + relative_path + "vehicle")
sys.path.append(abs_dir_path + relative_path + "obstacle")
sys.path.append(abs_dir_path + relative_path + "plan/dwa")

# import component modules
from global_xy_visualizer import GlobalXYVisualizer
from min_max import MinMax
from time_parameters import TimeParameters
from vehicle_specification import VehicleSpecification
from state import State
from four_wheels_vehicle import FourWheelsVehicle
from obstacle import Obstacle
from obstacle_list import ObstacleList
from dwa_controller import DwaController


# flag to show plot figure
# when executed as unit test, this flag is set as false
show_plot = True


def main():
    """
    Main process function
    """

    # set simulation parameters
    x_lim, y_lim = MinMax(-5, 55), MinMax(-15, 35)
    gif_path = abs_dir_path + relative_simulations + \
        "path_planning/dynamic_window_approach/dynamic_window_approach.gif"
    vis = GlobalXYVisualizer(x_lim, y_lim, TimeParameters(span_sec=25), show_zoom=False, gif_name=gif_path)

    # static obstacles between the start and the goal
    obst_list = ObstacleList()
    obst_list.add_obstacle(Obstacle(State(x_m=12.0, y_m=2.0), length_m=1.5, width_m=4.0))
    obst_list.add_obstacle(Obstacle(State(x_m=22.0, y_m=14.0), length_m=3.0, width_m=1.5))
    obst_list.add_obstacle(Obstacle(State(x_m=30.0, y_m=5.0, yaw_rad=np.deg2rad(30)), length_m=2.0, width_m=1.0))
    obst_list.add_obstacle(Obstacle(State(x_m=38.0, y_m=20.0), length_m=1.5, width_m=1.5))
    obst_list.add_obstacle(Obstacle(State(x_m=10.0, y_m=20.0), length_m=2.0, width_m=2.0))
    vis.add_object(obst_list)

    # create vehicle instance with DWA as its controller
    spec = VehicleSpecification()
    dwa = DwaController(spec, goal_xy=(45.0, 25.0), obst_list=obst_list)
    vehicle = FourWheelsVehicle(State(color=spec.color), spec, controller=dwa, show_zoom=False)
    vis.add_object(vehicle)

    # plot figure is not shown when executed as unit test
    if not show_plot: vis.not_show_plot()

    # show plot figure
    vis.draw()


# execute main process
if __name__ == "__main__":
    main()

"""
frenet_optimal_trajectory_planning.py

Title: Frenet Optimal Trajectory
Description: Samples polynomial trajectories along the course in the Frenet frame and follows the cheapest collision free one
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
sys.path.append(abs_dir_path + relative_path + "course/cubic_spline_course")
sys.path.append(abs_dir_path + relative_path + "plan/frenet_optimal_trajectory")

# import component modules
from global_xy_visualizer import GlobalXYVisualizer
from min_max import MinMax
from time_parameters import TimeParameters
from vehicle_specification import VehicleSpecification
from state import State
from four_wheels_vehicle import FourWheelsVehicle
from obstacle import Obstacle
from obstacle_list import ObstacleList
from cubic_spline_course import CubicSplineCourse
from frenet_optimal_trajectory_planner import FrenetOptimalTrajectoryPlanner


# flag to show plot figure
# when executed as unit test, this flag is set as false
show_plot = True


def main():
    """
    Main process function
    """

    # set simulation parameters
    x_lim, y_lim = MinMax(-5, 125), MinMax(-25, 25)
    gif_path = abs_dir_path + relative_simulations + \
        "path_planning/frenet_optimal_trajectory_planning/frenet_optimal_trajectory_planning.gif"
    vis = GlobalXYVisualizer(x_lim, y_lim, TimeParameters(span_sec=30), gif_name=gif_path)

    # reference course = center of the lane
    course = CubicSplineCourse([0.0, 20.0, 40.0, 60.0, 80.0, 100.0, 120.0],
                               [0.0, -2.0, 3.0, 5.0, 2.0, -3.0, 0.0],
                               20)
    vis.add_object(course)

    # static obstacles standing on the lane
    obst_list = ObstacleList()
    obst_list.add_obstacle(Obstacle(State(x_m=25.0, y_m=-1.0), length_m=1.5, width_m=1.0))
    obst_list.add_obstacle(Obstacle(State(x_m=50.0, y_m=6.0), length_m=1.5, width_m=1.0))
    obst_list.add_obstacle(Obstacle(State(x_m=75.0, y_m=1.0, yaw_rad=np.deg2rad(20)), length_m=2.0, width_m=1.0))
    obst_list.add_obstacle(Obstacle(State(x_m=95.0, y_m=-5.0), length_m=1.5, width_m=1.0))
    vis.add_object(obst_list)

    # create vehicle instance with the Frenet planner as its controller
    spec = VehicleSpecification(area_size=25.0)
    planner = FrenetOptimalTrajectoryPlanner(spec, course, obst_list)
    vehicle = FourWheelsVehicle(State(speed_mps=3.0, color=spec.color), spec, controller=planner)
    vis.add_object(vehicle)

    # plot figure is not shown when executed as unit test
    if not show_plot: vis.not_show_plot()

    # show plot figure
    vis.draw()


# execute main process
if __name__ == "__main__":
    main()

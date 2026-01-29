"""
rrt_star_path_planning.py

Author: Yuvraj Gupta
Simulation script for RRT* path planning.
"""

import matplotlib
matplotlib.use("Agg")

import numpy as np
import sys
from pathlib import Path
import json

# ---------------- Path setup ----------------
abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"
relative_simulations = "/../../../simulations/"

sys.path.append(abs_dir_path + relative_path + "visualization")
sys.path.append(abs_dir_path + relative_path + "state")
sys.path.append(abs_dir_path + relative_path + "vehicle")
sys.path.append(abs_dir_path + relative_path + "obstacle")
sys.path.append(abs_dir_path + relative_path + "mapping/grid")
sys.path.append(abs_dir_path + relative_path + "course/cubic_spline_course")
sys.path.append(abs_dir_path + relative_path + "control/pure_pursuit")
sys.path.append(abs_dir_path + relative_path + "plan/rrt")

# ---------------- Imports ----------------
from global_xy_visualizer import GlobalXYVisualizer
from min_max import MinMax
from time_parameters import TimeParameters
from vehicle_specification import VehicleSpecification
from state import State
from four_wheels_vehicle import FourWheelsVehicle
from obstacle import Obstacle
from obstacle_list import ObstacleList
from cubic_spline_course import CubicSplineCourse
from pure_pursuit_controller import PurePursuitController
from rrt_star_path_planner import RrtStarPathPlanner
from binary_occupancy_grid import BinaryOccupancyGrid

# ---------------- Config ----------------
show_plot = True


def main():
    # ---------------- Limits ----------------
    x_lim, y_lim = MinMax(-5, 55), MinMax(-20, 25)

    navigation_gif = (
        abs_dir_path
        + relative_simulations
        + "path_planning/rrt_star_path_planning/rrt_star_navigate.gif"
    )

    search_gif = (
        abs_dir_path
        + relative_simulations
        + "path_planning/rrt_star_path_planning/rrt_star_search.gif"
    )

    map_path = (
        abs_dir_path
        + relative_simulations
        + "path_planning/rrt_path_planning/map.json"
    )

    path_file = (
        abs_dir_path
        + relative_simulations
        + "path_planning/rrt_star_path_planning/path.json"
    )

    # ---------------- Visualizer ----------------
    vis = GlobalXYVisualizer(
        x_lim,
        y_lim,
        TimeParameters(span_sec=25),
        show_zoom=False,
        gif_name=navigation_gif,
    )

    occ_grid = BinaryOccupancyGrid(
        x_lim, y_lim, resolution=0.5, clearance=1.5, map_path=map_path
    )

    # ---------------- Obstacles ----------------
    obst_list = ObstacleList()
    obst_list.add_obstacle(Obstacle(State(x_m=10, y_m=15), 10, 8))
    obst_list.add_obstacle(Obstacle(State(x_m=40, y_m=0), 2, 10))
    obst_list.add_obstacle(
        Obstacle(State(x_m=10, y_m=-10, yaw_rad=np.deg2rad(45)), 5, 5)
    )
    obst_list.add_obstacle(
        Obstacle(State(x_m=30, y_m=15, yaw_rad=np.deg2rad(10)), 5, 2)
    )
    obst_list.add_obstacle(
        Obstacle(State(x_m=50, y_m=15, yaw_rad=np.deg2rad(15)), 5, 2)
    )
    obst_list.add_obstacle(Obstacle(State(x_m=25, y_m=0), 2, 2))
    obst_list.add_obstacle(Obstacle(State(x_m=35, y_m=-15), 7, 2))

    vis.add_object(obst_list)
    occ_grid.add_object(obst_list)
    occ_grid.save_map()

    # ---------------- RRT* ----------------
    planner = RrtStarPathPlanner(
        start=(0, 0),
        goal=(50, -10),
        map_file=map_path,
        x_lim=x_lim,
        y_lim=y_lim,
        path_filename=path_file,
        gif_name=search_gif,
        max_iterations=5000,
        step_size=0.5,
        goal_sample_rate=0.05,
    )

    # ---------------- Load path ----------------
    with open(path_file, "r") as f:
        sparse_path = json.load(f)

    sx = [p[0] for p in sparse_path]
    sy = [p[1] for p in sparse_path]

    course = CubicSplineCourse(sx, sy, 20)
    vis.add_object(course)

    # ---------------- Vehicle ----------------
    spec = VehicleSpecification()
    controller = PurePursuitController(spec, course)

    vehicle = FourWheelsVehicle(
        State(color=spec.color),
        spec,
        controller=controller,
        show_zoom=False,
    )

    vis.add_object(vehicle)

    if not show_plot:
        vis.not_show_plot()

    vis.draw()


if __name__ == "__main__":
    main()

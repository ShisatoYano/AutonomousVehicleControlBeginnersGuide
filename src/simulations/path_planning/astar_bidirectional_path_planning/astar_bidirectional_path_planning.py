"""
astar_bidirectional_path_planning.py

Author: Auto-generated
"""

# import path setting
import numpy as np
import sys
from pathlib import Path

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
sys.path.append(abs_dir_path + relative_path + "plan/astar_bidirectional")


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
from pure_pursuit_controller import PurePursuitController
from astar_bidirectional_path_planner import AStarBidirectionalPathPlanner
from binary_occupancy_grid import BinaryOccupancyGrid
import json


# flag to show plot figure
# when executed as unit test, this flag is set as false
show_plot = True


def main():
    """
    Main process function
    """

    # set simulation parameters
    x_lim, y_lim = MinMax(-5, 55), MinMax(-20, 25)
    navigation_gif_path = (
        abs_dir_path
        + relative_simulations
        + "path_planning/astar_bidirectional_path_planning/astar_bidirectional_navigate.gif"
    )
    map_path = (
        abs_dir_path
        + relative_simulations
        + "path_planning/astar_bidirectional_path_planning/map.json"
    )
    path_filename = (
        abs_dir_path
        + relative_simulations
        + "path_planning/astar_bidirectional_path_planning/path.json"
    )
    search_gif_path = (
        abs_dir_path
        + relative_simulations
        + "path_planning/astar_bidirectional_path_planning/astar_bidirectional_search.gif"
    )

    vis = GlobalXYVisualizer(
        x_lim,
        y_lim,
        TimeParameters(span_sec=25),
        show_zoom=False,
        gif_name=navigation_gif_path,
    )
    occ_grid = BinaryOccupancyGrid(
        x_lim, y_lim, resolution=0.5, clearance=1.5, map_path=map_path
    )

    obst_list = ObstacleList()
    # Map for bidirectional A*: obstacles funnel both searches toward the middle.
    # Start (0,0) and Goal (50,-10); meeting point ~(25, -5). Bidir expands fewer nodes.
    # Left half: block direct path from start, force flow toward center
    obst_list.add_obstacle(Obstacle(State(x_m=12.0, y_m=-12.0), length_m=8, width_m=2))
    obst_list.add_obstacle(Obstacle(State(x_m=10.0, y_m=6.0), length_m=6, width_m=9))
    # Right half: block direct path from goal, force flow toward center
    obst_list.add_obstacle(Obstacle(State(x_m=42.0, y_m=4.0), length_m=8, width_m=9))
    obst_list.add_obstacle(Obstacle(State(x_m=42.0, y_m=-14.0), length_m=6, width_m=5))
    # Central corridor stays clear so forward and backward meet in the middle
    obst_list.add_obstacle(Obstacle(State(x_m=25.0, y_m=10.0), length_m=10, width_m=8))
    obst_list.add_obstacle(Obstacle(State(x_m=25.0, y_m=-16.0), length_m=10, width_m=3))

    vis.add_object(obst_list)
    occ_grid.add_object(obst_list)
    occ_grid.save_map()
    # Easy Goal = (50,22)
    # Hard Goal = (50,-10)
    planner = AStarBidirectionalPathPlanner(
        (53, -15),
        (0, 0),
        map_path,
        weight=5.0,
        x_lim=x_lim,
        y_lim=y_lim,
        path_filename=path_filename,
        gif_name=search_gif_path,
    )

    # Load sparse path from json file
    with open(path_filename, "r") as f:
        sparse_path = json.load(f)

    # Extract x and y coordinates
    sparse_x = [point[0] for point in sparse_path]
    sparse_y = [point[1] for point in sparse_path]

    # Use with CubicSplineCourse
    course = CubicSplineCourse(sparse_x, sparse_y, 20)
    vis.add_object(course)

    # create vehicle instance
    spec = VehicleSpecification()
    pure_pursuit = PurePursuitController(spec, course)
    vehicle = FourWheelsVehicle(
        State(color=spec.color), spec, controller=pure_pursuit, show_zoom=False
    )

    vis.add_object(vehicle)

    # plot figure is not shown when executed as unit test
    if not show_plot:
        vis.not_show_plot()

    # show plot figure
    vis.draw()


# execute main process
if __name__ == "__main__":
    main()

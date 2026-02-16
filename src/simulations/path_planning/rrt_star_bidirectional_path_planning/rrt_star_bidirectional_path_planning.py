"""
rrt_star_bidirectional_path_planning.py

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
sys.path.append(abs_dir_path + relative_path + "plan/rrt_star_bidirectional")


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
from rrt_star_bidirectional_path_planner import RrtStarBidirectionalPathPlanner
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
    navigation_gif_path = abs_dir_path + relative_simulations + "path_planning/rrt_star_bidirectional_path_planning/rrt_star_bidirectional_navigate.gif"
    map_path = abs_dir_path + relative_simulations + "path_planning/rrt_star_bidirectional_path_planning/map.json"
    path_filename = abs_dir_path + relative_simulations + "path_planning/rrt_star_bidirectional_path_planning/path.json"
    search_gif_path = abs_dir_path + relative_simulations + "path_planning/rrt_star_bidirectional_path_planning/rrt_star_bidirectional_search.gif"


    vis = GlobalXYVisualizer(x_lim, y_lim, TimeParameters(span_sec=25), show_zoom=False, gif_name=navigation_gif_path)
    occ_grid = BinaryOccupancyGrid(x_lim, y_lim, resolution=0.5, clearance=1.5, map_path=map_path)

    obst_list = ObstacleList()
    obst_list.add_obstacle(Obstacle(State(x_m=10.0, y_m=15.0), length_m=10, width_m=8))
    obst_list.add_obstacle(Obstacle(State(x_m=40.0, y_m=0.0), length_m=2, width_m=10))
    obst_list.add_obstacle(Obstacle(State(x_m=10.0, y_m=-10.0, yaw_rad=np.rad2deg(45)), length_m=5, width_m=5))
    obst_list.add_obstacle(Obstacle(State(x_m=30.0, y_m=15.0, yaw_rad=np.rad2deg(10)), length_m=5, width_m=2))
    obst_list.add_obstacle(Obstacle(State(x_m=50.0, y_m=15.0, yaw_rad=np.rad2deg(15)), length_m=5, width_m=2))
    obst_list.add_obstacle(Obstacle(State(x_m=25.0, y_m=0.0), length_m=2, width_m=2))
    obst_list.add_obstacle(Obstacle(State(x_m=35.0, y_m=-15.0), length_m=7, width_m=2))

    vis.add_object(obst_list)
    occ_grid.add_object(obst_list)
    occ_grid.save_map()
    # Easy Goal = (50,22)
    # Hard Goal = (50,-10)
    planner = RrtStarBidirectionalPathPlanner((0, 0), (50, -10), map_path, x_lim=x_lim, y_lim=y_lim, path_filename=path_filename, gif_name=search_gif_path,
                                               max_iterations=5000, step_size=0.5, goal_sample_rate=0.05)

    # Load sparse path from json file
    with open(path_filename, 'r') as f:
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
    vehicle = FourWheelsVehicle(State(color=spec.color), spec,
                                controller=pure_pursuit,
                                show_zoom=False)

    vis.add_object(vehicle)

    # plot figure is not shown when executed as unit test
    if not show_plot: vis.not_show_plot()

    # show plot figure
    vis.draw()


# execute main process
if __name__ == "__main__":
    main()

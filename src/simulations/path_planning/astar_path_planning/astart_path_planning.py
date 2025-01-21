"""
ndt_map_construction.py

Author: Shisato Yano
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
sys.path.append(abs_dir_path + relative_path + "sensors")
sys.path.append(abs_dir_path + relative_path + "sensors/lidar")
sys.path.append(abs_dir_path + relative_path + "mapping/ndt")
sys.path.append(abs_dir_path + relative_path + "course/cubic_spline_course")
sys.path.append(abs_dir_path + relative_path + "control/pure_pursuit")
sys.path.append(abs_dir_path + relative_path + "plan/astar")


# import component modules
from global_xy_visualizer import GlobalXYVisualizer
from min_max import MinMax
from time_parameters import TimeParameters
from vehicle_specification import VehicleSpecification
from state import State
from four_wheels_vehicle import FourWheelsVehicle
from obstacle import Obstacle
from obstacle_list import ObstacleList
from sensors import Sensors
from sensor_parameters import SensorParameters
from omni_directional_lidar import OmniDirectionalLidar
from ndt_global_mapper import NdtGlobalMapper
from cubic_spline_course import CubicSplineCourse
from pure_pursuit_controller import PurePursuitController
from astar_path_planner import AStarPathPlanner


# flag to show plot figure
# when executed as unit test, this flag is set as false
show_plot = True


def main():
    """
    Main process function
    """

    # set simulation parameters
    x_lim, y_lim = MinMax(-5, 55), MinMax(-20, 25)
    vis = GlobalXYVisualizer(x_lim, y_lim, TimeParameters(span_sec=25), show_zoom=False)

    

    obstacle_parameters = [
    {"x_m": 10.0, "y_m": 15.0, "yaw_rad": 0.0, "length_m": 10.0, "width_m": 8.0},
    {"x_m": 40.0, "y_m": 0.0, "yaw_rad": 0.0, "length_m": 2.0, "width_m": 10.0},
    {"x_m": 10.0, "y_m": -10.0, "yaw_rad": np.rad2deg(45), "length_m": 5.0, "width_m": 5.0},
    {"x_m": 30.0, "y_m": 15.0, "yaw_rad": np.rad2deg(10), "length_m": 5.0, "width_m": 2.0},
    {"x_m": 50.0, "y_m": 15.0, "yaw_rad": np.rad2deg(15), "length_m": 5.0, "width_m": 2.0},
    {"x_m": 25.0, "y_m": 0.0, "yaw_rad": 0.0, "length_m": 2.0, "width_m": 2.0},
    {"x_m": 35.0, "y_m": -15.0, "yaw_rad": 0.0, "length_m": 7.0, "width_m": 2.0}
    ]

    # create obstacle instances
    obst_list = ObstacleList()
  
    for params in obstacle_parameters:
        obstacle = Obstacle(
            State(x_m=params["x_m"], y_m=params["y_m"], yaw_rad=params["yaw_rad"]),
            length_m=params["length_m"],
            width_m=params["width_m"]
        )
        obst_list.add_obstacle(obstacle)

    vis.add_object(obst_list)
    # Easy Goal = (50,22)
    # Hard Goal = (50,-10)
    planner = AStarPathPlanner((0, 0), (50, -10), obstacle_parameters, resolution=0.5, weight=5.0, obstacle_clearance = 1.5, visualize=True, x_lim=x_lim, y_lim=y_lim)
    path = planner.search()
    gif_path = abs_dir_path + relative_simulations + "path_planning/astar_path_planning/astar_search.gif"
    planner.visualize_search(path, gif_name=gif_path)

    sparse_path = planner.make_sparse_path(path, num_points=20)

    # Extract x and y coordinates
    sparse_x = [point[0] for point in sparse_path]
    sparse_y = [point[1] for point in sparse_path]

    print(sparse_x)
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

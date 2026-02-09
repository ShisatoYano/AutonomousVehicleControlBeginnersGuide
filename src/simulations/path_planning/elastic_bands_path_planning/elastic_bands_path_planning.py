"""
elastic_bands_path_planning.py

Simulation that demonstrates Elastic Bands path smoothing.

Two GIFs are produced:
    1. **elastic_bands_search.gif** – grid animation: A* initial search,
       initial path, then iterative elastic-bands optimisation with bubbles.
    2. **elastic_bands_navigate.gif** – car-following navigation on the
       smoothed path using PurePursuit.
"""

import numpy as np
import sys
import json
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
sys.path.append(abs_dir_path + relative_path + "plan/elastic_bands")

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
from binary_occupancy_grid import BinaryOccupancyGrid
from elastic_bands_path_planner import ElasticBandsPathPlanner


# flag to show plot figure
# when executed as unit test, this flag is set as false
show_plot = True


def main():
    """Main process function"""

    # set simulation parameters
    x_lim, y_lim = MinMax(-5, 55), MinMax(-20, 25)
    sim_dir = (abs_dir_path + relative_simulations
               + "path_planning/elastic_bands_path_planning/")
    map_path = sim_dir + "map.json"
    path_filename = sim_dir + "path.json"
    search_gif_path = sim_dir + "elastic_bands_search.gif"
    navigate_gif_path = sim_dir + "elastic_bands_navigate.gif"

    # ---- grid + static obstacles ----
    occ_grid = BinaryOccupancyGrid(x_lim, y_lim, resolution=0.5,
                                   clearance=1.5, map_path=map_path)

    obst_list = ObstacleList()
    obst_list.add_obstacle(Obstacle(State(x_m=10.0, y_m=15.0),
                                    length_m=10, width_m=8))
    obst_list.add_obstacle(Obstacle(State(x_m=40.0, y_m=0.0),
                                    length_m=2, width_m=10))
    occ_grid.add_object(obst_list)
    occ_grid.save_map()

    # ---- Elastic Bands plan ----
    planner = ElasticBandsPathPlanner(
        (0, 0), (50, -10), map_path,
        x_lim=x_lim, y_lim=y_lim,
        path_filename=path_filename,
        gif_name=search_gif_path if show_plot else None,
    )

    if not planner.path:
        print("Elastic Bands: no path found – aborting.")
        return

    # ---- navigation GIF (car following) ----
    with open(path_filename, 'r') as f:
        sparse_path = json.load(f)

    sparse_x = [p[0] for p in sparse_path]
    sparse_y = [p[1] for p in sparse_path]

    course = CubicSplineCourse(sparse_x, sparse_y, 20)

    vis = GlobalXYVisualizer(x_lim, y_lim, TimeParameters(span_sec=25),
                             show_zoom=False, gif_name=navigate_gif_path)
    vis.add_object(obst_list)
    vis.add_object(course)

    spec = VehicleSpecification()
    pure_pursuit = PurePursuitController(spec, course)
    vehicle = FourWheelsVehicle(State(color=spec.color), spec,
                                controller=pure_pursuit, show_zoom=False)
    vis.add_object(vehicle)

    if not show_plot:
        vis.not_show_plot()
    vis.draw()


if __name__ == "__main__":
    main()

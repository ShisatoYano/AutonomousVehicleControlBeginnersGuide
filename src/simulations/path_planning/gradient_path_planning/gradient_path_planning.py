"""
gradient_path_planning.py

Author: Panav Arpit Raaj

Demo: build a static potential field, export to JSON, plan a path
using gradient descent, then simulate a vehicle following it.
"""

import sys
import json
import numpy as np
from pathlib import Path

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"

sys.path.append(abs_dir_path + relative_path + "mapping/potential")
sys.path.append(abs_dir_path + relative_path + "mapping/grid")
sys.path.append(abs_dir_path + relative_path + "plan/gradient")
sys.path.append(abs_dir_path + relative_path + "visualization")
sys.path.append(abs_dir_path + relative_path + "state")
sys.path.append(abs_dir_path + relative_path + "vehicle")
sys.path.append(abs_dir_path + relative_path + "obstacle")
sys.path.append(abs_dir_path + relative_path + "course/cubic_spline_course")
sys.path.append(abs_dir_path + relative_path + "control/pure_pursuit")

from potential_field_map import PotentialFieldMap
from gradient_path_planner import GradientPathPlanner
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

show_plot = True


def main():
    # -- Build potential field --
    pf_map = PotentialFieldMap(
        width_m=60.0, height_m=45.0, resolution_m=0.5,
        center_x_m=25.0, center_y_m=2.5,
        zeta=0.1, eta=200.0, rho=5.0,
        goal_x_m=50.0, goal_y_m=-13.0
    )

    obstacle_centers = [
        (10.0, 15.0), (40.0, 0.0), (10.0, -10.0),
        (30.0, 15.0), (50.0, 15.0), (25.0, 0.0), (35.0, -15.0)
    ]
    for ox, oy in obstacle_centers:
        for x in np.arange(ox - 2, ox + 3, 0.5):
            for y in np.arange(oy - 2, oy + 3, 0.5):
                pf_map.update_map([x], [y])

    # -- Export to JSON and plan path --
    json_path = abs_dir_path + "/potential_field.json"
    pf_map.export_to_json(json_path)

    planner = GradientPathPlanner(
        potential_file=json_path,
        start=(0.0, 0.0),
        step_size=0.3,
        goal_tolerance=1.0
    )
    result = planner.plan()

    path_json = abs_dir_path + "/gradient_path.json"
    planner.save_path(result['path'], path_json)

    # -- Load path and create smooth course --
    with open(path_json, 'r') as f:
        sparse_path = json.load(f)
    sparse_x = [point[0] for point in sparse_path]
    sparse_y = [point[1] for point in sparse_path]

    course = CubicSplineCourse(sparse_x, sparse_y, 20)

    # -- Vehicle navigation simulation --
    x_lim, y_lim = MinMax(-5, 55), MinMax(-20, 25)
    gif_path = abs_dir_path + "/gradient_descent_demo.gif"
    vis = GlobalXYVisualizer(x_lim, y_lim, TimeParameters(span_sec=25),
                             show_zoom=False, gif_name=gif_path)

    # Add heatmap as background (wrap draw_map to match visualizer interface)
    class _HeatmapWrapper:
        def __init__(self, pf): self.pf = pf
        def draw(self, axes, elems): self.pf.draw_map(axes, elems)

    vis.add_object(_HeatmapWrapper(pf_map))
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

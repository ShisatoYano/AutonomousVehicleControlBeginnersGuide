"""
lidar_obstacle_sensing.py

Author: Shisato Yano
"""

import numpy as np
import sys
from pathlib import Path

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../"
sys.path.append(abs_dir_path + relative_path + "visualization")
sys.path.append(abs_dir_path + relative_path + "state")
sys.path.append(abs_dir_path + relative_path + "vehicle")
sys.path.append(abs_dir_path + relative_path + "obstacle")
sys.path.append(abs_dir_path + relative_path + "sensor/lidar")

from global_xy_visualizer import GlobalXYVisualizer
from min_max import MinMax
from time_parameters import TimeParameters
from vehicle_specification import VehicleSpecification
from state import State
from four_wheels_vehicle import FourWheelsVehicle
from obstacle import Obstacle
from obstacle_list import ObstacleList
from omni_directional_lidar import OmniDirectionalLidar


show_plot = True


def main():
    x_lim, y_lim = MinMax(-30, 30), MinMax(-30, 30)
    vis = GlobalXYVisualizer(x_lim, y_lim, TimeParameters(span_sec=20))

    # obstacle instances
    obst_list = ObstacleList()
    obst1 = Obstacle(State(x_m=-5.0, y_m=15.0, speed_mps=1.0), yaw_rate_rps=np.deg2rad(10), width_m=1.0)
    obst_list.add_obstacle(obst1)
    obst2 = Obstacle(State(x_m=-15.0, y_m=-15.0), length_m=10.0, width_m=5.0)
    obst_list.add_obstacle(obst2)
    obst3 = Obstacle(State(x_m=20.0), yaw_rate_rps=np.deg2rad(15))
    obst_list.add_obstacle(obst3)
    vis.add_object(obst_list)

    # vehicle instance
    spec = VehicleSpecification(area_size=30.0)
    vehicle_state = State(color=spec.color)
    lidar = OmniDirectionalLidar(obst_list, inst_lon_m=spec.wheel_base_m/2)
    vehicle = FourWheelsVehicle(vehicle_state, spec, sensor=lidar)
    vis.add_object(vehicle)

    if not show_plot: vis.not_show_plot()

    vis.draw()


if __name__ == "__main__":
    main()

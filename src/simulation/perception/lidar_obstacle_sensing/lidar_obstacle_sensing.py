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
from vehicle_specification import VehicleSpecification
from state import State
from state_history import StateHistory
from four_wheels_vehicle import FourWheelsVehicle
from obstacle import Obstacle
from obstacle_list import ObstacleList
from omni_directional_lidar import OmniDirectionalLidar


show_plot = True


def main():
    vis = GlobalXYVisualizer(x_min=-30, x_max=30, y_min=-30, y_max=30, time_span_s=20)

    # obstacle instances
    obst_list = ObstacleList()
    obst1 = Obstacle(State(-5.0, 15.0, 0.0, 1.0), yaw_rate_rps=np.deg2rad(10), width_m=1.0)
    obst_list.add_obstacle(obst1)
    obst2 = Obstacle(State(-15.0, -15.0, 0.0, 0.0), length_m=10.0, width_m=5.0)
    obst_list.add_obstacle(obst2)
    obst3 = Obstacle(State(20.0, 0.0, 0.0, 0.0), yaw_rate_rps=np.deg2rad(15))
    obst_list.add_obstacle(obst3)
    vis.add_object(obst_list)

    # vehicle instance
    spec = VehicleSpecification()
    vehicle_state = State(0.0, 0.0, 0.0, 0.0)
    history = StateHistory([vehicle_state.get_x_m()], [vehicle_state.get_y_m()], spec.color)
    lidar = OmniDirectionalLidar(obst_list, inst_lon_m=spec.wheel_base_m/2)
    vehicle = FourWheelsVehicle(vehicle_state, history, spec,
                                sensor=lidar,
                                draw_area_width=30.0)
    vis.add_object(vehicle)

    if not show_plot: vis.not_show_plot()

    vis.draw()


if __name__ == "__main__":
    main()

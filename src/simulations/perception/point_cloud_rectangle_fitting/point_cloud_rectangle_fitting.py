"""
point_cloud_rectangle_fitting.py

Author: Shisato Yano
"""

# import path setting
import numpy as np
import sys
from pathlib import Path

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"

sys.path.append(abs_dir_path + relative_path + "visualization")
sys.path.append(abs_dir_path + relative_path + "state")
sys.path.append(abs_dir_path + relative_path + "vehicle")
sys.path.append(abs_dir_path + relative_path + "obstacle")
sys.path.append(abs_dir_path + relative_path + "sensors")
sys.path.append(abs_dir_path + relative_path + "sensors/lidar")


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


# flag to show plot figure
# when executed as unit test, this flag is set as false
show_plot = True


def main():
    """
    Main process function
    """
    
    # set simulation parameters
    x_lim, y_lim = MinMax(-30, 30), MinMax(-30, 30)
    vis = GlobalXYVisualizer(x_lim, y_lim, TimeParameters(span_sec=20))

    # create obstacle instances
    obst_list = ObstacleList()
    obst1 = Obstacle(State(x_m=-5.0, y_m=15.0, speed_mps=1.0), yaw_rate_rps=np.deg2rad(10), width_m=1.0)
    obst_list.add_obstacle(obst1)
    obst2 = Obstacle(State(x_m=-15.0, y_m=-15.0), length_m=10.0, width_m=5.0)
    obst_list.add_obstacle(obst2)
    obst3 = Obstacle(State(x_m=20.0), yaw_rate_rps=np.deg2rad(15))
    obst_list.add_obstacle(obst3)
    vis.add_object(obst_list)

    # create vehicle instance
    spec = VehicleSpecification(area_size=30.0) # spec instance
    lidar = OmniDirectionalLidar(obst_list, SensorParameters(lon_m=spec.wheel_base_m/2)) # lidar instance
    vehicle = FourWheelsVehicle(State(color=spec.color), spec, sensors=Sensors(lidar=lidar)) # set state, spec, lidar as arguments
    vis.add_object(vehicle)

    # plot figure is not shown when executed as unit test
    if not show_plot: vis.not_show_plot()

    # show plot figure
    vis.draw()


# execute main process
if __name__ == "__main__":
    main()

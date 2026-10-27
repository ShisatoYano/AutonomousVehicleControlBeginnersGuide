"""
vfh_dynamic_speed_control.py

Title: VFH Dynamic Speed Control
Description: Slows the VFH controller's target speed as obstacle density rises in the vehicle's steering direction
Author: Khushi
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
sys.path.append(abs_dir_path + relative_path + "mapping/polar_histogram")
sys.path.append(abs_dir_path + relative_path + "control/vfh")


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
from polar_histogram_mapper import PolarHistogramMapper
from vfh_controller import VfhController


# flag to show plot figure
# when executed as unit test, this flag is set as false
show_plot = True


def main():
    """
    Main process function
    """

    # set simulation parameters
    x_lim, y_lim = MinMax(-30, 30), MinMax(-30, 30)
    vis = GlobalXYVisualizer(x_lim, y_lim, TimeParameters(span_sec=25))

    # create obstacle instances
    # same scenario as vfh_polar_histogram_construction(Step 1) through
    # vfh_vehicle_motion_integration(Step 4), so this step's driven speed
    # can be compared directly against those steps' constant-speed output
    obst_list = ObstacleList()
    obst1 = Obstacle(State(x_m=-5.0, y_m=15.0, speed_mps=1.0), yaw_rate_rps=np.deg2rad(10), width_m=1.0)
    obst_list.add_obstacle(obst1)
    obst2 = Obstacle(State(x_m=-15.0, y_m=-15.0), length_m=10.0, width_m=5.0)
    obst_list.add_obstacle(obst2)
    obst3 = Obstacle(State(x_m=20.0), yaw_rate_rps=np.deg2rad(15))
    obst_list.add_obstacle(obst3)
    vis.add_object(obst_list)

    # create vehicle instance with the Step 3 mapper for sensing plus a
    # VfhController whose min_speed_mps/danger_density/caution_half_angle_rad
    # now scale target speed down as obst3 is approached and steered around
    # (Step 5), instead of holding cruise_speed_mps constant like Step 4
    spec = VehicleSpecification(area_size=30.0)  # spec instance
    sensor_params = SensorParameters(lon_m=spec.wheel_base_m/2)
    lidar = OmniDirectionalLidar(obst_list, sensor_params)  # lidar instance
    mapper = PolarHistogramMapper(sensor_params=sensor_params, num_sectors=72,
                                  smoothing_window=5, valley_density_threshold=0.2,
                                  target_x_m=25.0, target_y_m=5.0,
                                  target_weight=1.0, heading_weight=0.5,
                                  previous_weight=0.8)  # Step 3 mapper instance
    controller = VfhController(spec, mapper, cruise_speed_mps=3.0,
                               speed_gain=1.0, yaw_rate_gain=1.5,
                               max_yaw_rate_rps=1.0, min_speed_mps=0.5,
                               danger_density=1.0,
                               caution_half_angle_rad=np.deg2rad(10))  # Step 5 controller instance
    vehicle = FourWheelsVehicle(State(color=spec.color), spec, controller=controller,
                                sensors=Sensors(lidar=lidar), mapper=mapper)  # set state, spec, controller, lidar, mapper as arguments
    vis.add_object(vehicle)

    # plot figure is not shown when executed as unit test
    if not show_plot: vis.not_show_plot()

    # show plot figure
    vis.draw()


# execute main process
if __name__ == "__main__":
    main()

"""
icp_scan_matching.py

Title: ICP Scan Matching Odometry
Description: Estimates the vehicle's motion only from LiDAR by aligning each scan to the previous one with Iterative Closest Point
Author: Yuvraj Gupta
"""

# import path setting
import sys
from pathlib import Path
import numpy as np

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"
relative_simulations = "/../../../simulations/"

sys.path.append(abs_dir_path + relative_path + "visualization")
sys.path.append(abs_dir_path + relative_path + "state")
sys.path.append(abs_dir_path + relative_path + "vehicle")
sys.path.append(abs_dir_path + relative_path + "obstacle")
sys.path.append(abs_dir_path + relative_path + "course/cubic_spline_course")
sys.path.append(abs_dir_path + relative_path + "control/pure_pursuit")
sys.path.append(abs_dir_path + relative_path + "sensors")
sys.path.append(abs_dir_path + relative_path + "sensors/lidar")
sys.path.append(abs_dir_path + relative_path + "localization/icp")

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
from sensors import Sensors
from sensor_parameters import SensorParameters
from omni_directional_lidar import OmniDirectionalLidar
from scan_matching_odometry import ScanMatchingOdometry


# flag to show plot figure
# when executed as unit test, this flag is set as false
show_plot = True


def main():
    """
    Main process function
    """

    # seed the random number generator of the LiDAR noise for reproducible runs
    np.random.seed(0)

    # set simulation parameters
    x_lim, y_lim = MinMax(-10, 60), MinMax(-25, 30)
    gif_path = abs_dir_path + relative_simulations + "localization/icp_scan_matching/icp_scan_matching.gif"
    vis = GlobalXYVisualizer(x_lim, y_lim, TimeParameters(span_sec=25), show_zoom=False, gif_name=gif_path)

    # course which the vehicle follows
    course = CubicSplineCourse([0.0, 10.0, 25.0, 40.0, 50.0],
                               [0.0, 4.0, -8.0, 10.0, 0.0],
                               15)
    vis.add_object(course)

    # static obstacles give the LiDAR something to see
    obst_list = ObstacleList()
    obst_list.add_obstacle(Obstacle(State(x_m=5.0, y_m=15.0), length_m=6.0, width_m=1.0))
    obst_list.add_obstacle(Obstacle(State(x_m=20.0, y_m=-18.0), length_m=8.0, width_m=1.0))
    obst_list.add_obstacle(Obstacle(State(x_m=30.0, y_m=18.0, yaw_rad=np.deg2rad(20)), length_m=5.0, width_m=1.5))
    obst_list.add_obstacle(Obstacle(State(x_m=-5.0, y_m=-8.0), length_m=1.5, width_m=4.0))
    obst_list.add_obstacle(Obstacle(State(x_m=50.0, y_m=-12.0), length_m=2.0, width_m=2.0))
    obst_list.add_obstacle(Obstacle(State(x_m=55.0, y_m=15.0, yaw_rad=np.deg2rad(45)), length_m=3.0, width_m=1.0))
    vis.add_object(obst_list)

    # vehicle with a LiDAR installed at the rear axle center
    spec = VehicleSpecification()
    lidar = OmniDirectionalLidar(obst_list, SensorParameters())
    sensors = Sensors(lidar=lidar)
    pure_pursuit = PurePursuitController(spec, course)
    vehicle = FourWheelsVehicle(State(color=spec.color), spec, controller=pure_pursuit,
                                sensors=sensors, show_zoom=False)
    vis.add_object(vehicle)

    # ICP odometry reads the same LiDAR, so it is added after the vehicle
    odometry = ScanMatchingOdometry(sensors)
    vis.add_object(odometry)

    # plot figure is not shown when executed as unit test
    if not show_plot: vis.not_show_plot()

    # show plot figure
    vis.draw()


# execute main process
if __name__ == "__main__":
    main()

"""
point_cloud_rectangle_fitting.py

Author: Shreyansh Shethia
"""

# import path setting
import numpy as np
import sys
from pathlib import Path
import matplotlib.pyplot as plt

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"
relative_simulations = "/../../../simulations/"

sys.path.append(abs_dir_path + relative_path + "visualization")
sys.path.append(abs_dir_path + relative_path + "state")
sys.path.append(abs_dir_path + relative_path + "vehicle")
sys.path.append(abs_dir_path + relative_path + "obstacle")
sys.path.append(abs_dir_path + relative_path + "sensors")
sys.path.append(abs_dir_path + relative_path + "sensors/lidar")
sys.path.append(abs_dir_path + relative_path + "detection/l_shape_fitting")


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
from l_shape_fitting_detector import LShapeFittingDetector
from robust_least_squares_detector import OptimizedLShapeDetector, DualLShapeDetector, tracker #RansacOptimizationDetector #RobustRectangleFittingDetector

# flag to show plot figure
# when executed as unit test, this flag is set as false
show_plot = True
# --- STEP 1: Define at the top ---

def main():
    """
    Main process function
    """
    
    # set simulation parameters
    x_lim, y_lim = MinMax(-30, 30), MinMax(-30, 30)
    navigation_gif_path = abs_dir_path + relative_simulations + "perception/point_cloud_rectangle_fitting/point_cloud_robustLS_rectangle_fitting.gif"
    vis = GlobalXYVisualizer(x_lim, y_lim, TimeParameters(span_sec=20), gif_name=navigation_gif_path)

    # create obstacle instances
    obst_list = ObstacleList()
    obst1 = Obstacle(State(x_m=-5.0, y_m=15.0, speed_mps=1.0), yaw_rate_rps=np.deg2rad(10), width_m=1.0)
    obst_list.add_obstacle(obst1)
    #obst2 = Obstacle(State(x_m=0.0, y_m=-15.0), length_m=10.0, width_m=5.0)
    #obst_list.add_obstacle(obst2)
    obst3 = Obstacle(State(x_m=20.0), yaw_rate_rps=np.deg2rad(15))
    obst_list.add_obstacle(obst3)
    #obst4 = Obstacle(State(x_m=-20.0, y_m=0.0), width_m=1.0)
    #obst_list.add_obstacle(obst4)
    obst6 = Obstacle(State(x_m=-15.0, y_m=-15.0), length_m=10.0, width_m=5.0)
    obst_list.add_obstacle(obst6)
    #obst7 = Obstacle(State(x_m=-15.0, y_m=15.0, yaw_rad = np.pi/4), length_m=10.0, width_m=5.0)
    #obst_list.add_obstacle(obst7)
    vis.add_object(obst_list)

    # create vehicle instance
    spec = VehicleSpecification(area_size=30.0) # spec instance
    lidar = OmniDirectionalLidar(obst_list, SensorParameters(lon_m=spec.wheel_base_m/2)) # lidar instance
    detector = DualLShapeDetector(show_logger_debug=True if show_plot else False) # detector instance 
    vehicle = FourWheelsVehicle(State(color=spec.color), spec, sensors=Sensors(lidar=lidar), detector=detector) # set state, spec, lidar, detector as arguments
    vis.add_object(vehicle)

    # plot figure is not shown when executed as unit test
    if not show_plot:
        vis.not_show_plot()
    else:
        print("Simulation started. Close the simulation window to generate report after simulation.")

    # ... Execute simulation ...
    vis.draw()
    if (show_plot and len(obst_list.get_list())==1):
        # This only runs AFTER you close the simulation window
        # If there are multiple obstacles, the plot gets too crowded to be useful
        print("Generating post-simulation report...")
        tracker.plot()

    


# execute main process
if __name__ == "__main__":
    main()

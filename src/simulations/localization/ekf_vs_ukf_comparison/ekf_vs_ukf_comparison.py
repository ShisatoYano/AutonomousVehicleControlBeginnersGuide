"""
ekf_vs_ukf_comparison.py

Author: Bruno DOKPOMIWA

This script tries to compare Extended Kalman Filter (EKF) and Unscented Kalman Filter (UKF)
for vehicle localization.
"""

# import path setting
import sys
from pathlib import Path

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"

sys.path.append(abs_dir_path + relative_path + "visualization")
sys.path.append(abs_dir_path + relative_path + "state")
sys.path.append(abs_dir_path + relative_path + "vehicle")
sys.path.append(abs_dir_path + relative_path + "course/cubic_spline_course")
sys.path.append(abs_dir_path + relative_path + "control/pure_pursuit")
sys.path.append(abs_dir_path + relative_path + "sensors")
sys.path.append(abs_dir_path + relative_path + "sensors/gnss")
sys.path.append(abs_dir_path + relative_path + "localization/kalman_filter")


# import component modules
from global_xy_visualizer import GlobalXYVisualizer
from min_max import MinMax
from time_parameters import TimeParameters
from vehicle_specification import VehicleSpecification
from state import State
from four_wheels_vehicle import FourWheelsVehicle
from cubic_spline_course import CubicSplineCourse
from pure_pursuit_controller import PurePursuitController
from sensors import Sensors
from gnss import Gnss
from extended_kalman_filter_localizer import ExtendedKalmanFilterLocalizer
from unscented_kalman_filter_localizer import UnscentedKalmanFilterLocalizer


# flag to show plot figure
# when executed as unit test, this flag is set as false
show_plot = True


def main():
    """
    Main process function
    Comparison between EKF and UKF localization
    """
    
    # set simulation parameters
    x_lim, y_lim = MinMax(-5, 55), MinMax(-20, 25)
    vis = GlobalXYVisualizer(x_lim, y_lim, TimeParameters(span_sec=30))

    # create course data instance (trajectory)
    course = CubicSplineCourse([0.0, 10.0, 25, 40, 50],
                               [0.0, 4, -12, 20, -13],
                               20)
    vis.add_object(course)

    # create vehicle's spec instance
    spec = VehicleSpecification(area_size=20.0)
    
    gnss = Sensors(gnss=Gnss(x_noise_std=1.0, y_noise_std=1.0, color='g'))
    
    # create EKF vehicle
    ekf_state = State(color='b', x_m=0.0, y_m=0.0, yaw_rad=0.0, speed_mps=0.0)
    ekf_controller = PurePursuitController(spec, course, color='m')
    ekf_localizer = ExtendedKalmanFilterLocalizer(color='r')
    ekf_vehicle = FourWheelsVehicle(ekf_state, spec, controller=ekf_controller, 
                                    sensors=gnss, localizer=ekf_localizer)
    vis.add_object(ekf_vehicle)
    
    # create UKF vehicle
    ukf_state = State(color='c', x_m=0.0, y_m=0.0, yaw_rad=0.0, speed_mps=0.0)
    ukf_controller = PurePursuitController(spec, course, color='m')
    ukf_localizer = UnscentedKalmanFilterLocalizer(color='orange')
    ukf_vehicle = FourWheelsVehicle(ukf_state, spec, controller=ukf_controller, 
                                     sensors=gnss, localizer=ukf_localizer)
    vis.add_object(ukf_vehicle)

    # plot figure is not shown when executed as unit test
    if not show_plot: vis.not_show_plot()

    # show plot figure
    vis.draw()


# execute main process
if __name__ == "__main__":
    main()

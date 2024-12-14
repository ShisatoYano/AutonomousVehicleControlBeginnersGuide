"""
rear_wheel_feedback_tracking.py

Author: Shisato Yano
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
sys.path.append(abs_dir_path + relative_path + "control/rear_wheel_feedback")

# import component modules
from global_xy_visualizer import GlobalXYVisualizer
from min_max import MinMax
from time_parameters import TimeParameters
from vehicle_specification import VehicleSpecification
from state import State
from four_wheels_vehicle import FourWheelsVehicle
from cubic_spline_course import CubicSplineCourse
from rear_wheel_feedback_controller import RearWheelFeedbackController


# flag to show plot figure
# when executed as unit test, this flag is set as false
show_plot = True


def main():
    """
    Main process function
    """

    # set simulation parameters
    x_lim, y_lim = MinMax(-5, 55), MinMax(-20, 25)
    vis = GlobalXYVisualizer(x_lim, y_lim, TimeParameters(span_sec=25))


# execute main process
if __name__ == "__main__":
    main()

"""
adaptive_pure_pursuit_path_tracking.py

Path tracking simulation using Adaptive Pure Pursuit controller.
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

# import component modules
from global_xy_visualizer import GlobalXYVisualizer
from min_max import MinMax
from time_parameters import TimeParameters
from vehicle_specification import VehicleSpecification
from state import State
from four_wheels_vehicle import FourWheelsVehicle
from cubic_spline_course import CubicSplineCourse
from adaptive_pure_pursuit_controller import AdaptivePurePursuitController

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

    # create course data instance
    course = CubicSplineCourse([0.0, 10.0, 25, 40, 50],
                               [0.0, 4, -12, 20, -13],
                               20)
    vis.add_object(course)

    # create vehicle's spec instance
    spec = VehicleSpecification(area_size=20.0)

    # create vehicle's state instance
    state = State(color=spec.color)

    # create adaptive pure pursuit controller instance
    adaptive_pure_pursuit = AdaptivePurePursuitController(
        spec,
        course,
        color='g',
        min_look_ahead_m=2.0,
        look_forward_gain=0.3,
        curvature_adapt_gain=1.0,
        max_look_ahead_m=15.0,
    )

    # create vehicle instance
    vehicle = FourWheelsVehicle(state, spec, controller=adaptive_pure_pursuit)
    vis.add_object(vehicle)

    # plot figure is not shown when executed as unit test
    if not show_plot:
        vis.not_show_plot()

    # show plot figure
    vis.draw()


# execute main process
if __name__ == "__main__":
    main()

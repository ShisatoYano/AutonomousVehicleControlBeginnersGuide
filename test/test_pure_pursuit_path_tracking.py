"""
Test of Path tracking simulation by Pure pursuit algorithm

Author: Shisato Yano
"""

from pathlib import Path
import sys
import pytest
import numpy as np

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/path_tracking/pure_pursuit_path_tracking")
import pure_pursuit_path_tracking


# Half of a narrow 3 m lane, about twice the largest error measured with this course
MAX_LATERAL_ERROR_M = 1.5
# The controller stops short of the last point by its look-ahead; about twice the measured distance
MAX_DISTANCE_TO_END_M = 5.0


def test_simulation():
    pure_pursuit_path_tracking.show_plot = False

    vehicle, course = pure_pursuit_path_tracking.main()

    course_x_m, course_y_m = np.array(course.x_array), np.array(course.y_array)
    lateral_errors_m = [
        np.min(np.hypot(course_x_m - x_m, course_y_m - y_m))
        for x_m, y_m in zip(vehicle.state.x_history, vehicle.state.y_history)
    ]
    assert max(lateral_errors_m) < MAX_LATERAL_ERROR_M

    distance_to_end_m = np.hypot(vehicle.state.get_x_m() - course_x_m[-1], vehicle.state.get_y_m() - course_y_m[-1])
    assert distance_to_end_m < MAX_DISTANCE_TO_END_M
    assert vehicle.state.get_speed_mps() == 0.0

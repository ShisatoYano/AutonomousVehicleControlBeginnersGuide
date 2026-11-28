"""
Test of path tracking simulation by Adaptive Pure Pursuit algorithm
"""

from pathlib import Path
import sys
import pytest

from path_tracking_checks import assert_course_tracked

sys.path.append(
    str(Path(__file__).absolute().parent)
    + "/../src/simulations/path_tracking/adaptive_pure_pursuit_path_tracking"
)
import adaptive_pure_pursuit_path_tracking


# About twice the values measured with this course (0.88 m, 2.40 m and 0.17 m/s)
MAX_LATERAL_ERROR_M = 1.8
# The controller stops short of the last point by its look-ahead
MAX_DISTANCE_TO_END_M = 5.0
# The simulation ends while the vehicle is still slowing down, just before it stops
MAX_FINAL_SPEED_MPS = 0.5


def test_simulation():
    adaptive_pure_pursuit_path_tracking.show_plot = False

    vehicle, course = adaptive_pure_pursuit_path_tracking.main()

    assert_course_tracked(vehicle, course, MAX_LATERAL_ERROR_M, MAX_DISTANCE_TO_END_M, MAX_FINAL_SPEED_MPS)

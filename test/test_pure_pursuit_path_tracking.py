"""
Test of Path tracking simulation by Pure pursuit algorithm

Author: Shisato Yano
"""

from pathlib import Path
import sys
import pytest

from path_tracking_checks import assert_course_tracked

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/path_tracking/pure_pursuit_path_tracking")
import pure_pursuit_path_tracking


# Half of a narrow 3 m lane, about twice the largest error measured with this course
MAX_LATERAL_ERROR_M = 1.5
# The controller stops short of the last point by its look-ahead; about twice the measured distance
MAX_DISTANCE_TO_END_M = 5.0


def test_simulation():
    pure_pursuit_path_tracking.show_plot = False

    vehicle, course = pure_pursuit_path_tracking.main()

    assert_course_tracked(vehicle, course, MAX_LATERAL_ERROR_M, MAX_DISTANCE_TO_END_M)

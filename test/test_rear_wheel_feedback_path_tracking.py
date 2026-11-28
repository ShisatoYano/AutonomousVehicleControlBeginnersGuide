"""
Test of Path tracking simulation by Rear wheel feedback algorithm

Author: Shisato Yano
"""

from pathlib import Path
import sys
import pytest

from path_tracking_checks import assert_course_tracked

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/path_tracking/rear_wheel_feedback_tracking")
import rear_wheel_feedback_tracking


# About twice the largest error measured with this course (0.48 m)
MAX_LATERAL_ERROR_M = 1.0
# The vehicle stops 0.004 m from the last point; 1 m leaves room for small numerical differences
MAX_DISTANCE_TO_END_M = 1.0


def test_simulation():
    rear_wheel_feedback_tracking.show_plot = False

    vehicle, course = rear_wheel_feedback_tracking.main()

    assert_course_tracked(vehicle, course, MAX_LATERAL_ERROR_M, MAX_DISTANCE_TO_END_M)

"""
Test of path tracking simulation by MPC (Model Predictive Control) algorithm
"""

from pathlib import Path
import sys
import pytest

from path_tracking_checks import assert_course_tracked

sys.path.append(
    str(Path(__file__).absolute().parent)
    + "/../src/simulations/path_tracking/mpc_path_tracking"
)

import mpc_path_tracking


# About twice the largest error measured with this course (0.99 m)
MAX_LATERAL_ERROR_M = 2.0
# The vehicle stops 0.01 m from the last point; 1 m leaves room for small numerical differences
MAX_DISTANCE_TO_END_M = 1.0


def test_simulation():
    mpc_path_tracking.show_plot = False

    vehicle, course = mpc_path_tracking.main()

    assert_course_tracked(vehicle, course, MAX_LATERAL_ERROR_M, MAX_DISTANCE_TO_END_M)

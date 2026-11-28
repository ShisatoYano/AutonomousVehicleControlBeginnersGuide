"""
Test of path tracking simulation by MPPI (Model Predictive Path Integral) algorithm
"""

from pathlib import Path
import sys
import pytest

from path_tracking_checks import assert_course_tracked

sys.path.append(
    str(Path(__file__).absolute().parent)
    + "/../src/simulations/path_tracking/mppi_path_tracking"
)
import mppi_path_tracking


# About twice the values measured with this course and seed (1.31 m, 1.50 m and 0.52 m/s)
MAX_LATERAL_ERROR_M = 2.6
MAX_DISTANCE_TO_END_M = 3.0
# The simulation ends while the vehicle is still slowing down, just before it stops
MAX_FINAL_SPEED_MPS = 1.0


def test_simulation():
    mppi_path_tracking.show_plot = False

    vehicle, course = mppi_path_tracking.main()

    assert_course_tracked(vehicle, course, MAX_LATERAL_ERROR_M, MAX_DISTANCE_TO_END_M, MAX_FINAL_SPEED_MPS)

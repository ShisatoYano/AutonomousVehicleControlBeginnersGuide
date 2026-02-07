"""
Test of path tracking simulation by MPPI (Model Predictive Path Integral) algorithm
"""

from pathlib import Path
import sys
import pytest

sys.path.append(
    str(Path(__file__).absolute().parent)
    + "/../src/simulations/path_tracking/mppi_path_tracking"
)
import mppi_path_tracking


def test_simulation():
    mppi_path_tracking.show_plot = False
    mppi_path_tracking.main()

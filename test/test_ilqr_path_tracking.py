"""
Test of Path tracking simulation by iLQR (iterative Linear Quadratic Regulator) algorithm

Author: Yuvraj Gupta
"""

from pathlib import Path
import sys
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/path_tracking/ilqr_path_tracking")
import ilqr_path_tracking


def test_simulation():
    ilqr_path_tracking.show_plot = False

    ilqr_path_tracking.main()

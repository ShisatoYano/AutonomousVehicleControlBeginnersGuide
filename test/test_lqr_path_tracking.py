"""
Test of Path tracking simulation by LQR(Linear Quadratic Regulator) algorithm

Author: Shisato Yano
"""

from pathlib import Path
import sys
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/path_tracking/lqr_path_tracking")
import lqr_path_tracking


def test_simulation():
    lqr_path_tracking.show_plot = False

    lqr_path_tracking.main()

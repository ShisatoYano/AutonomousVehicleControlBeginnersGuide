"""
Test of 1 dimensional position estimation Simulation by Linear Kalman Filter

Author: Mohammed Shoaib Choudry
"""

from pathlib import Path
import sys
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/localization/kalman_filter_1d")
import kalman_filter_1d_pos


def test_simulation():
    kalman_filter_1d_pos.show_plot = False

    kalman_filter_1d_pos.main()

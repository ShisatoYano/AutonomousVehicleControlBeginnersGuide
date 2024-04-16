"""
Test of Localization and Path tracking Simulation by EKF and Pure pursuit

Author: Shisato Yano
"""

from pathlib import Path
import sys
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/localization/extended_kalman_filter_localization")
import extended_kalman_filter_localization


def test_simulation():
    extended_kalman_filter_localization.show_plot = False

    extended_kalman_filter_localization.main()

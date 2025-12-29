"""
Test of Localization and Path tracking Simulation by UKF and Pure pursuit

Author: Bruno DOKPOMIWA
"""

from pathlib import Path
import sys
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/localization/unscented_kalman_filter_localization")
import unscented_kalman_filter_localization


def test_simulation():
    unscented_kalman_filter_localization.show_plot = False

    unscented_kalman_filter_localization.main()


"""
Test of EKF vs UKF Localization Comparison Simulation

Author: Bruno DOKPOMIWA
"""

from pathlib import Path
import sys
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/localization/ekf_vs_ukf_comparison")
import ekf_vs_ukf_comparison


def test_simulation():
    ekf_vs_ukf_comparison.show_plot = False

    ekf_vs_ukf_comparison.main()


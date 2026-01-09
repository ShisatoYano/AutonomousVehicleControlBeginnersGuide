"""
Test of EKF vs UKF vs PF Localization Comparison Simulation

Author: Sahruday Patti
"""

from pathlib import Path
import sys
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/localization/ekf_ukf_pf_comparison")
import ekf_ukf_pf_comparison


def test_simulation():
    ekf_ukf_pf_comparison.show_plot = False

    ekf_ukf_pf_comparison.main()


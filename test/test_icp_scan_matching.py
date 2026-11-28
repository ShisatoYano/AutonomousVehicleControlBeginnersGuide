"""
Test of ICP scan matching odometry simulation

Author: Yuvraj Gupta
"""

from pathlib import Path
import sys
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/localization/icp_scan_matching")
import icp_scan_matching


def test_simulation():
    icp_scan_matching.show_plot = False

    icp_scan_matching.main()

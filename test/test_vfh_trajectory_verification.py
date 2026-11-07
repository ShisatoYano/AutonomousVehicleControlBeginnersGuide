"""
Test of VFH trajectory verification simulation

Author: Khushi
"""

from pathlib import Path
import sys

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/mapping/vfh_trajectory_verification")
import vfh_trajectory_verification


def test_simulation():
    vfh_trajectory_verification.show_plot = False

    vfh_trajectory_verification.main()

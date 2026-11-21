"""
Test of VFH vehicle motion integration simulation

Author: Khushi
"""

from pathlib import Path
import sys

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/mapping/vfh_vehicle_motion_integration")
import vfh_vehicle_motion_integration


def test_simulation():
    vfh_vehicle_motion_integration.show_plot = False

    vfh_vehicle_motion_integration.main()

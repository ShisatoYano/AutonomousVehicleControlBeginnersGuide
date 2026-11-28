"""
Test of VFH dynamic speed control simulation

Author: Khushi
"""

from pathlib import Path
import sys

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/mapping/vfh_dynamic_speed_control")
import vfh_dynamic_speed_control


def test_simulation():
    vfh_dynamic_speed_control.show_plot = False

    vfh_dynamic_speed_control.main()

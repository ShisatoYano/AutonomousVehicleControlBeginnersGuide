"""
Test of VFH direction selection simulation

Author: Khushi
"""

from pathlib import Path
import sys

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/mapping/vfh_direction_selection")
import vfh_direction_selection


def test_simulation():
    vfh_direction_selection.show_plot = False

    vfh_direction_selection.main()

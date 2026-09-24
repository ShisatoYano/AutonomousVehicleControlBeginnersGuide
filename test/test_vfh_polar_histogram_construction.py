"""
Test of VFH polar histogram construction simulation

Author: Khushi
"""

from pathlib import Path
import sys

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/mapping/vfh_polar_histogram_construction")
import vfh_polar_histogram_construction


def test_simulation():
    vfh_polar_histogram_construction.show_plot = False

    vfh_polar_histogram_construction.main()

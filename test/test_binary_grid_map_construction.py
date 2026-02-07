"""
Test of Binary grid map construction

Author: Shisato Yano
Updated by: Bhavesh Lokesh Agarwal
"""

from pathlib import Path
import sys
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/mapping/binary_grid_map_construction")
import binary_grid_map_construction


def test_simulation():
    binary_grid_map_construction.show_plot = False

    binary_grid_map_construction.main()


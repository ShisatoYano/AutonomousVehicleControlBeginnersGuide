"""
Test of Cost grid map construction

Author: Bhavesh Lokesh Agarwal
"""

from pathlib import Path
import sys
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/mapping/cost_grid_map_construction")
import cost_grid_map_construction


def test_simulation():
    cost_grid_map_construction.show_plot = False

    cost_grid_map_construction.main()


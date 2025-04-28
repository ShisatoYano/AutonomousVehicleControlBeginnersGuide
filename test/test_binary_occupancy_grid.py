"""
Test of Binary Map Construction

Author: Shantanu Parab
"""

from pathlib import Path
import sys
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/mapping/grid")
import binary_occupancy_grid


def test_simulation():
    binary_occupancy_grid.show_plot = False
    binary_occupancy_grid.main()

"""
Test of Map construction by NDT algorithm

Author: Shisato Yano
"""

from pathlib import Path
import sys
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/mapping/ndt_map_construction")
import ndt_map_construction


def test_simulation():
    ndt_map_construction.show_plot = False

    ndt_map_construction.main()

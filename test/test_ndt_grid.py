"""
Unit test of NdtGrid

Author: Shisato Yano
"""

import pytest
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/mapping/ndt")
from ndt_grid import NdtGrid


grid = NdtGrid()


def test_initialization():
    assert grid.points_num == 0
    assert grid.mean_x_m == None
    assert grid.mean_y_m == None
    assert grid.covariance == None

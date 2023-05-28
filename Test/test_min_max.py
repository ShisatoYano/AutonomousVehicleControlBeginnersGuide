"""
Unit test of MinMax

Author: Shisato Yano
"""

import pytest
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/visualization")
from min_max import MinMax


def test_initialize():
    min_max = MinMax()

    assert min_max.min_value() == 0
    assert min_max.max_value() == 10


def test_set_argument():
    min_max = MinMax(-10, 100)

    assert min_max.min_value() == -10
    assert min_max.max_value() == 100

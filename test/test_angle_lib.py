"""
Unit test of Angle Lib functions

Author: Shisato Yano
"""

import pytest
import sys
import numpy as np
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/common")
from angle_lib import pi_to_pi


def test_lower_than_pi():
    assert pi_to_pi(0.0) == 0.0
    assert pi_to_pi(np.pi / 2) == np.pi / 2
    assert pi_to_pi(np.pi) == -np.pi

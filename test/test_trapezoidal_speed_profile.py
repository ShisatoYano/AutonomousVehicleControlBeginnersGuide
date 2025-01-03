"""
Unit test of TrapezoidalSpeedProfile

Author: Shisato Yano
"""

import pytest
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../src/components/control/speed_profile")
from trapezoidal_speed_profile import TrapezoidalSpeedProfile


# test instance
spd_prf = TrapezoidalSpeedProfile(10, 2, 100)


def test_init():
    assert spd_prf.max_spd_mps == 10
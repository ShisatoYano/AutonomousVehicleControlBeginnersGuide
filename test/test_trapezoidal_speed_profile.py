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
    assert spd_prf.max_accel_mps2 == 2
    assert spd_prf.distance_m == 100

    assert spd_prf.accel_time_s == 5
    assert spd_prf.decel_time_s == 5
    assert spd_prf.const_time_s == 5

    assert spd_prf.target_speed_mps == 0.0
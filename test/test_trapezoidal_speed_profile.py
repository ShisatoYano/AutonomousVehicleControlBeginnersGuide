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


def test_target_speed_accel_time():
    assert spd_prf.decide_target_speed_mps(0.0, 1.0) == 2.0
    assert spd_prf.decide_target_speed_mps(1.0, 1.0) == 4.0
    assert spd_prf.decide_target_speed_mps(2.0, 1.0) == 6.0
    assert spd_prf.decide_target_speed_mps(3.0, 1.0) == 8.0
    assert spd_prf.decide_target_speed_mps(4.0, 1.0) == 10.0
    assert spd_prf.decide_target_speed_mps(5.0, 1.0) == 10.0


def test_target_speed_const_time():
    assert spd_prf.decide_target_speed_mps(6.0, 1.0) == 10.0
    assert spd_prf.decide_target_speed_mps(7.0, 1.0) == 10.0
    assert spd_prf.decide_target_speed_mps(8.0, 1.0) == 10.0
    assert spd_prf.decide_target_speed_mps(9.0, 1.0) == 10.0
    assert spd_prf.decide_target_speed_mps(10.0, 1.0) == 10.0

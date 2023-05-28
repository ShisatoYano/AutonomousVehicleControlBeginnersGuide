"""
Unit test of TimeParameters

Author: Shisato Yano
"""

import pytest
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/visualization")
from time_parameters import TimeParameters


# test instance
param = TimeParameters()


def test_initialize():
    assert param.span_sec == 10
    assert param.interval_sec == 0.1
    assert param.interval_msec == 100
    assert param.frame_num == 101


def test_get_interval_sec():
    assert param.get_interval_sec() == 0.1


def test_get_interval_msec():
    assert param.get_interval_msec() == 100


def test_get_frame_num():
    assert param.get_frame_num() == 101


def test_current_sec():
    assert param.current_sec(0) == 0
    assert param.current_sec(10) == 1
    assert param.current_sec(100) == 10


def test_simulation_finished():
    assert param.simulation_finished(0) == False
    assert param.simulation_finished(50) == False
    assert param.simulation_finished(100) == True

"""
Unit test of VfhController

Author: Khushi
"""

from math import atan2
import pytest
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/state")
sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/vehicle")
sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/control/vfh")
from state import State
from vehicle_specification import VehicleSpecification
from vfh_controller import VfhController


class _FakeDirectionSelector:
    """
    Minimal stand-in for DirectionSelector exposing only the getter
    VfhController actually reads, so the controller can be unit tested
    without building a full PolarHistogramMapper/valley pipeline
    """

    def __init__(self, angle_rad):
        self.angle_rad = angle_rad

    def get_selected_angle_rad(self):
        return self.angle_rad


class _FakeMapper:
    """
    Minimal stand-in for a mapper exposing only get_direction_selector()
    """

    def __init__(self, angle_rad):
        self.selector = _FakeDirectionSelector(angle_rad)

    def get_direction_selector(self):
        return self.selector

    def set_selected_angle_rad(self, angle_rad):
        self.selector.angle_rad = angle_rad


def _spec():
    return VehicleSpecification()  # wheel_base_m = 2.0 by default


def test_invalid_cruise_speed_raises():
    with pytest.raises(ValueError):
        VfhController(_spec(), _FakeMapper(0.0), cruise_speed_mps=-1.0)


def test_invalid_gains_raise():
    with pytest.raises(ValueError):
        VfhController(_spec(), _FakeMapper(0.0), speed_gain=-1.0)
    with pytest.raises(ValueError):
        VfhController(_spec(), _FakeMapper(0.0), yaw_rate_gain=-1.0)


def test_invalid_max_yaw_rate_raises():
    with pytest.raises(ValueError):
        VfhController(_spec(), _FakeMapper(0.0), max_yaw_rate_rps=0.0)
    with pytest.raises(ValueError):
        VfhController(_spec(), _FakeMapper(0.0), max_yaw_rate_rps=-1.0)


def test_falls_back_to_vehicle_heading_when_no_direction_selected():
    mapper = _FakeMapper(None)
    state = State(yaw_rad=0.3, speed_mps=1.0)
    controller = VfhController(_spec(), mapper, cruise_speed_mps=3.0,
                               speed_gain=1.0, yaw_rate_gain=2.0,
                               max_yaw_rate_rps=1.0)

    controller.update(state, 0.1)

    assert controller.get_target_angle_rad() == pytest.approx(0.3)
    assert controller.get_target_yaw_rate_rps() == pytest.approx(0.0)
    assert controller.get_target_steer_rad() == pytest.approx(0.0)
    assert controller.get_target_accel_mps2() == pytest.approx(2.0)


def test_accelerates_toward_cruise_speed_when_below_target():
    mapper = _FakeMapper(0.0)
    state = State(yaw_rad=0.0, speed_mps=0.0)
    controller = VfhController(_spec(), mapper, cruise_speed_mps=3.0, speed_gain=1.0)

    controller.update(state, 0.1)

    assert controller.get_target_accel_mps2() == pytest.approx(3.0)


def test_decelerates_toward_cruise_speed_when_above_target():
    mapper = _FakeMapper(0.0)
    state = State(yaw_rad=0.0, speed_mps=5.0)
    controller = VfhController(_spec(), mapper, cruise_speed_mps=3.0, speed_gain=1.0)

    controller.update(state, 0.1)

    assert controller.get_target_accel_mps2() == pytest.approx(-2.0)


def test_yaw_rate_proportional_to_heading_error():
    mapper = _FakeMapper(0.5)
    state = State(yaw_rad=0.0, speed_mps=2.0)
    controller = VfhController(_spec(), mapper, yaw_rate_gain=2.0, max_yaw_rate_rps=10.0)

    controller.update(state, 0.1)

    assert controller.get_target_yaw_rate_rps() == pytest.approx(1.0)


def test_yaw_rate_saturates_at_configured_maximum():
    mapper = _FakeMapper(2.0)
    state = State(yaw_rad=0.0, speed_mps=2.0)
    controller = VfhController(_spec(), mapper, yaw_rate_gain=5.0, max_yaw_rate_rps=1.0)

    controller.update(state, 0.1)
    assert controller.get_target_yaw_rate_rps() == pytest.approx(1.0)

    mapper.set_selected_angle_rad(-2.0)
    controller.update(state, 0.1)
    assert controller.get_target_yaw_rate_rps() == pytest.approx(-1.0)


def test_steer_angle_derived_from_yaw_rate_and_speed():
    mapper = _FakeMapper(1.0)
    state = State(yaw_rad=0.0, speed_mps=4.0)
    controller = VfhController(_spec(), mapper, yaw_rate_gain=1.0, max_yaw_rate_rps=10.0)

    controller.update(state, 0.1)

    expected_yaw_rate_rps = 1.0  # gain(1.0) * diff_angle(1.0), unsaturated
    expected_steer_rad = atan2(_spec().wheel_base_m * expected_yaw_rate_rps, 4.0)
    assert controller.get_target_yaw_rate_rps() == pytest.approx(expected_yaw_rate_rps)
    assert controller.get_target_steer_rad() == pytest.approx(expected_steer_rad)


def test_steer_is_zero_when_speed_is_near_zero():
    mapper = _FakeMapper(1.0)
    state = State(yaw_rad=0.0, speed_mps=0.0)
    controller = VfhController(_spec(), mapper, yaw_rate_gain=1.0, max_yaw_rate_rps=10.0)

    controller.update(state, 0.1)

    assert controller.get_target_steer_rad() == pytest.approx(0.0)


def test_get_target_angle_rad_returns_selected_direction():
    mapper = _FakeMapper(1.2)
    state = State(yaw_rad=0.0, speed_mps=1.0)
    controller = VfhController(_spec(), mapper)

    controller.update(state, 0.1)

    assert controller.get_target_angle_rad() == pytest.approx(1.2)


def test_draw_does_not_raise():
    controller = VfhController(_spec(), _FakeMapper(0.0))
    elems = []

    controller.draw(None, elems)

    assert elems == []

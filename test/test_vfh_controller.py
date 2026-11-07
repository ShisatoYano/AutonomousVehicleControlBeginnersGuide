"""
Unit test of VfhController

Author: Khushi
"""

from math import atan2, pi
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


class _FakeHistogram:
    """
    Minimal stand-in for PolarHistogram exposing only the query
    VfhController actually reads(Step 5: Dynamic Speed Control), so
    density-based speed control can be unit tested without building a
    real sector/density array. Remembers its last call's arguments so
    tests can confirm the controller queries the right direction
    """

    def __init__(self, density_ahead=0.0):
        self.density_ahead = density_ahead
        self.last_center_angle_rad = None
        self.last_half_width_rad = None

    def max_density_in_angle_range(self, center_angle_rad, half_width_rad=0.0):
        self.last_center_angle_rad = center_angle_rad
        self.last_half_width_rad = half_width_rad
        return self.density_ahead


class _FakeMapper:
    """
    Minimal stand-in for a mapper exposing only get_direction_selector()
    and get_histogram()
    """

    def __init__(self, angle_rad, density_ahead=0.0):
        self.selector = _FakeDirectionSelector(angle_rad)
        self.histogram = _FakeHistogram(density_ahead)

    def get_direction_selector(self):
        return self.selector

    def get_histogram(self):
        return self.histogram

    def set_selected_angle_rad(self, angle_rad):
        self.selector.angle_rad = angle_rad

    def set_density_ahead(self, density_ahead):
        self.histogram.density_ahead = density_ahead


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


def test_invalid_min_speed_raises():
    with pytest.raises(ValueError):
        VfhController(_spec(), _FakeMapper(0.0), cruise_speed_mps=3.0, min_speed_mps=-1.0)
    with pytest.raises(ValueError):
        VfhController(_spec(), _FakeMapper(0.0), cruise_speed_mps=3.0, min_speed_mps=4.0)


def test_invalid_danger_density_raises():
    with pytest.raises(ValueError):
        VfhController(_spec(), _FakeMapper(0.0), danger_density=0.0)
    with pytest.raises(ValueError):
        VfhController(_spec(), _FakeMapper(0.0), danger_density=-1.0)


def test_invalid_caution_half_angle_raises():
    with pytest.raises(ValueError):
        VfhController(_spec(), _FakeMapper(0.0), caution_half_angle_rad=-0.1)


def test_target_speed_is_cruise_speed_with_no_obstacle_ahead():
    mapper = _FakeMapper(0.0, density_ahead=0.0)
    state = State(yaw_rad=0.0, speed_mps=3.0)
    controller = VfhController(_spec(), mapper, cruise_speed_mps=3.0)

    controller.update(state, 0.1)

    assert controller.get_target_speed_mps() == pytest.approx(3.0)


def test_target_speed_drops_to_minimum_at_danger_density():
    mapper = _FakeMapper(0.0, density_ahead=1.0)
    state = State(yaw_rad=0.0, speed_mps=3.0)
    controller = VfhController(_spec(), mapper, cruise_speed_mps=3.0,
                               min_speed_mps=0.5, danger_density=1.0)

    controller.update(state, 0.1)

    assert controller.get_target_speed_mps() == pytest.approx(0.5)


def test_target_speed_scales_linearly_between_cruise_and_minimum():
    mapper = _FakeMapper(0.0, density_ahead=0.5)
    state = State(yaw_rad=0.0, speed_mps=3.0)
    controller = VfhController(_spec(), mapper, cruise_speed_mps=3.0,
                               min_speed_mps=1.0, danger_density=1.0)

    controller.update(state, 0.1)

    # halfway to danger_density -> halfway from cruise(3.0) down to minimum(1.0)
    assert controller.get_target_speed_mps() == pytest.approx(2.0)


def test_target_speed_does_not_go_below_minimum_past_danger_density():
    mapper = _FakeMapper(0.0, density_ahead=5.0)  # far past danger_density
    state = State(yaw_rad=0.0, speed_mps=3.0)
    controller = VfhController(_spec(), mapper, cruise_speed_mps=3.0,
                               min_speed_mps=0.5, danger_density=1.0)

    controller.update(state, 0.1)

    assert controller.get_target_speed_mps() == pytest.approx(0.5)


def test_acceleration_targets_the_scaled_down_speed():
    mapper = _FakeMapper(0.0, density_ahead=1.0)
    state = State(yaw_rad=0.0, speed_mps=0.0)
    controller = VfhController(_spec(), mapper, cruise_speed_mps=3.0, speed_gain=1.0,
                               min_speed_mps=0.5, danger_density=1.0)

    controller.update(state, 0.1)

    # target speed is scaled down to min_speed_mps(0.5), not cruise_speed_mps(3.0)
    assert controller.get_target_accel_mps2() == pytest.approx(0.5)


def test_queries_density_relative_to_vehicle_heading_in_target_direction():
    mapper = _FakeMapper(pi / 2, density_ahead=0.0)  # target direction, global frame
    state = State(yaw_rad=pi / 4, speed_mps=1.0)  # vehicle heading, global frame
    controller = VfhController(_spec(), mapper, caution_half_angle_rad=0.2)

    controller.update(state, 0.1)

    # target direction relative to heading = pi/2 - pi/4 = pi/4
    assert mapper.histogram.last_center_angle_rad == pytest.approx(pi / 4)
    assert mapper.histogram.last_half_width_rad == pytest.approx(0.2)

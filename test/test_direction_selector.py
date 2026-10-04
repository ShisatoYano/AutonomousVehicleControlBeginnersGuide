"""
Unit test of DirectionSelector

Author: Khushi
"""

import numpy as np
import pytest
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/mapping/polar_histogram")
from candidate_valley_detector import Valley
from direction_selector import DirectionSelector


def _valley_at(center_angle_deg):
    """
    Helper to build a minimal Valley with only its center angle set, since
    DirectionSelector only reads get_center_angle_rad()
    """

    return Valley(start_index=0, end_index=0, width_sectors=1,
                  center_angle_rad=np.deg2rad(center_angle_deg))


def test_invalid_weight_raises():
    with pytest.raises(ValueError):
        DirectionSelector(target_weight=-1.0)


def test_selected_angle_starts_as_none():
    selector = DirectionSelector()

    assert selector.get_selected_angle_rad() is None


def test_no_valleys_returns_none():
    selector = DirectionSelector()

    result = selector.select([], vehicle_yaw_rad=0.0, target_angle_rad=0.0)

    assert result is None
    assert selector.get_selected_angle_rad() is None


def test_single_valley_is_always_selected():
    selector = DirectionSelector()
    valleys = [_valley_at(45)]

    result = selector.select(valleys, vehicle_yaw_rad=0.0, target_angle_rad=np.deg2rad(-170))

    assert np.isclose(result, np.deg2rad(45))


def test_selects_valley_closest_to_target_direction():
    selector = DirectionSelector(target_weight=1.0, heading_weight=0.0, previous_weight=0.0)
    valleys = [_valley_at(0), _valley_at(90)]

    result = selector.select(valleys, vehicle_yaw_rad=0.0, target_angle_rad=np.deg2rad(10))

    assert np.isclose(result, np.deg2rad(0))


def test_selects_valley_closest_to_current_heading():
    selector = DirectionSelector(target_weight=0.0, heading_weight=1.0, previous_weight=0.0)
    valleys = [_valley_at(10), _valley_at(90)]

    # target strongly favours the 90[deg] valley, but only heading_weight
    # is active, so the candidate closest to straight-ahead(0[deg]) wins
    result = selector.select(valleys, vehicle_yaw_rad=0.0, target_angle_rad=np.deg2rad(90))

    assert np.isclose(result, np.deg2rad(10))


def test_valley_angle_is_converted_from_vehicle_relative_to_global():
    selector = DirectionSelector(target_weight=1.0, heading_weight=0.0, previous_weight=0.0)
    # vehicle faces global 90[deg]; the only valley is straight ahead of
    # the vehicle(0[deg] relative), which should resolve to global 90[deg]
    valleys = [_valley_at(0)]

    result = selector.select(valleys, vehicle_yaw_rad=np.deg2rad(90), target_angle_rad=np.deg2rad(90))

    assert np.isclose(result, np.deg2rad(90))


def test_first_call_uses_vehicle_heading_as_previous_direction():
    # with no prior selection, previous_weight should pull toward the
    # vehicle's current heading, same as heading_weight would alone
    selector = DirectionSelector(target_weight=0.0, heading_weight=0.0, previous_weight=1.0)
    valleys = [_valley_at(5), _valley_at(90)]

    result = selector.select(valleys, vehicle_yaw_rad=0.0, target_angle_rad=0.0)

    assert np.isclose(result, np.deg2rad(5))


def test_previous_direction_can_override_heading_preference():
    selector = DirectionSelector(target_weight=0.0, heading_weight=1.0, previous_weight=5.0)

    # first call: vehicle faces global 0[deg], two valleys straight ahead
    # of it, at relative 0[deg](-> global 0) and 90[deg](-> global 90).
    # heading_weight alone would prefer global 0(closer to current heading)
    first = selector.select([_valley_at(0), _valley_at(90)],
                            vehicle_yaw_rad=0.0, target_angle_rad=0.0)
    assert np.isclose(first, np.deg2rad(0))

    # second call: the vehicle has turned to face global 90[deg]. The same
    # two physical gaps are now at relative -90[deg](-> global 0, same as
    # the previous selection) and 0[deg](-> global 90, straight ahead now).
    # heading_weight alone would switch to global 90, but a strong
    # previous_weight should keep global 0 selected instead
    second = selector.select([_valley_at(-90), _valley_at(0)],
                             vehicle_yaw_rad=np.deg2rad(90), target_angle_rad=0.0)

    assert np.isclose(second, np.deg2rad(0))


def test_angle_wraps_correctly_near_boundary():
    selector = DirectionSelector(target_weight=1.0, heading_weight=0.0, previous_weight=0.0)
    # a valley at 170[deg] is actually close(20[deg]) to a target at
    # -170[deg] going the short way around, despite the raw numbers
    # looking far apart
    valleys = [_valley_at(170), _valley_at(0)]

    result = selector.select(valleys, vehicle_yaw_rad=0.0, target_angle_rad=np.deg2rad(-170))

    assert np.isclose(result, np.deg2rad(170))

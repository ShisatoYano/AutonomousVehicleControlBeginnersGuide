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
    Helper to build a minimal 1-sector-wide Valley, since that keeps it
    "narrow"(width_sectors <= DirectionSelector's default s_max_sectors)
    regardless of center angle, so DirectionSelector only reads
    get_center_angle_rad() and get_width_sectors() for these
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
def test_narrow_valley_still_uses_only_its_center():
    # width_sectors(4) at or below the default s_max_sectors(5) stays
    # "narrow": even though the target is outside it, no border
    # candidates are added - the single candidate is still just the center
    selector = DirectionSelector(target_weight=1.0, heading_weight=0.0, previous_weight=0.0)
    narrow_valley = Valley(start_index=0, end_index=0, width_sectors=4,
                           center_angle_rad=np.deg2rad(10))

    result = selector.select([narrow_valley], vehicle_yaw_rad=0.0, target_angle_rad=np.deg2rad(60))

    assert np.isclose(result, np.deg2rad(10))


def test_wide_valley_offers_both_border_candidates_when_target_is_outside_it():
    # width_sectors=20, sector_angle_rad defaults to 5[deg]: half_span =
    # 20*5/2 = 50[deg], so the two border-inset candidates
    # (s_max_sectors/2 = 2.5 sectors = 12.5[deg] in from each border) sit
    # at -37.5[deg] and +37.5[deg] from the center(0[deg])
    selector = DirectionSelector(target_weight=1.0, heading_weight=0.0, previous_weight=0.0)
    wide_valley = Valley(start_index=0, end_index=0, width_sectors=20,
                         center_angle_rad=0.0)

    # target(60[deg]) is outside the valley's -50..+50[deg] span, so it
    # is not itself offered as a candidate; only the two border-inset
    # candidates are, and the one closer to the target(+37.5[deg]) wins
    result = selector.select([wide_valley], vehicle_yaw_rad=0.0, target_angle_rad=np.deg2rad(60))

    assert np.isclose(result, np.deg2rad(37.5))


def test_full_circle_valley_selects_target_straight_ahead():
    """
    Regression test for the maintainer's first PR #82 review repro: with
    no obstacles at all, candidate valley detection(Step 2) returns one
    valley spanning the entire circle(72 sectors), whose center sits
    roughly opposite the target. Before this wide-valley handling, only
    that center was ever offered as a candidate, so a target straight
    ahead(0[deg]) could never actually be selected
    """

    selector = DirectionSelector()
    full_circle_valley = Valley(start_index=0, end_index=71, width_sectors=72,
                                center_angle_rad=np.deg2rad(180))

    result = selector.select([full_circle_valley], vehicle_yaw_rad=0.0, target_angle_rad=0.0)

    assert np.isclose(result, 0.0)


def test_wide_valley_selects_free_target_instead_of_its_center():
    """
    Regression test for the maintainer's second PR #82 review repro: one
    small obstacle behind the vehicle(sectors 40-44 of 72 blocked) leaves
    one wide valley whose center sits at 32.5[deg], even though the
    target direction(0[deg]) is itself free and inside that same valley
    """

    selector = DirectionSelector()
    wide_valley = Valley(start_index=45, end_index=39, width_sectors=67,
                         center_angle_rad=np.deg2rad(32.5))

    result = selector.select([wide_valley], vehicle_yaw_rad=0.0, target_angle_rad=0.0)

    assert np.isclose(result, 0.0)

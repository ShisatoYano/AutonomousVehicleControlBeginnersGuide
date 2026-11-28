"""
Unit test of the look-ahead distance of AdaptivePurePursuitController

Equation (8) of the controller:
l_d = min(max((k * v + l_d_min) / (1 + k_kappa * |kappa_n|), l_d_min), l_d_max)

Author: abedinm
"""

import sys
from pathlib import Path

import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/control/pure_pursuit")
from adaptive_pure_pursuit_controller import AdaptivePurePursuitController

# Gains used in every test: l_d_min = 2 m, k = 0.5 s, k_kappa = 1 m, l_d_max = 15 m
MIN_LOOK_AHEAD_M = 2.0
LOOK_FORWARD_GAIN = 0.5
CURVATURE_ADAPT_GAIN = 1.0
MAX_LOOK_AHEAD_M = 15.0


class FakeSpec:
    """Only the wheel base is read by the constructor."""

    wheel_base_m = 2.5


class FakeState:
    """Vehicle state that only reports its speed."""

    def __init__(self, speed_mps: float) -> None:
        self.speed_mps = speed_mps

    def get_speed_mps(self) -> float:
        return self.speed_mps


class FakeCourse:
    """Course whose nearest point always has the same curvature."""

    def __init__(self, curvature: float) -> None:
        self.curvature = curvature

    def search_nearest_point_index(self, state: FakeState) -> int:
        return 0

    def point_curvature(self, index: int) -> float:
        return self.curvature


class FakeCourseWithoutCurvature:
    """Course that has no point_curvature(), so kappa_n counts as 0."""

    def search_nearest_point_index(self, state: FakeState) -> int:
        return 0


def look_ahead_m(course: object, speed_mps: float) -> float:
    """Return the look-ahead distance [m] the controller computes for this course and speed."""
    controller = AdaptivePurePursuitController(
        FakeSpec(), course,
        min_look_ahead_m=MIN_LOOK_AHEAD_M,
        look_forward_gain=LOOK_FORWARD_GAIN,
        curvature_adapt_gain=CURVATURE_ADAPT_GAIN,
        max_look_ahead_m=MAX_LOOK_AHEAD_M,
    )
    controller._calculate_look_ahead_distance(FakeState(speed_mps))
    return controller.look_ahead_distance_m


def test_straight_course_look_ahead_is_k_times_speed_plus_minimum() -> None:
    # 0.5 * 10 + 2 = 7 m
    assert look_ahead_m(FakeCourse(0.0), 10.0) == pytest.approx(7.0)


def test_course_without_curvature_is_treated_as_straight() -> None:
    assert look_ahead_m(FakeCourseWithoutCurvature(), 10.0) == pytest.approx(7.0)


def test_look_ahead_at_speed_zero_is_the_minimum() -> None:
    assert look_ahead_m(FakeCourse(0.0), 0.0) == pytest.approx(MIN_LOOK_AHEAD_M)


def test_look_ahead_at_high_speed_is_capped_at_the_maximum() -> None:
    # 0.5 * 40 + 2 = 22 m, above the 15 m cap
    assert look_ahead_m(FakeCourse(0.0), 40.0) == pytest.approx(MAX_LOOK_AHEAD_M)


def test_larger_curvature_gives_shorter_look_ahead_at_the_same_speed() -> None:
    # Base at 20 m/s: 0.5 * 20 + 2 = 12 m, divided by 1 + |kappa|
    on_gentle_curve = look_ahead_m(FakeCourse(0.5), 20.0)
    on_sharp_curve = look_ahead_m(FakeCourse(2.0), 20.0)

    assert on_gentle_curve == pytest.approx(12.0 / 1.5)
    assert on_sharp_curve == pytest.approx(12.0 / 3.0)
    assert on_sharp_curve < on_gentle_curve


def test_negative_curvature_shortens_look_ahead_like_positive() -> None:
    # Only |kappa| counts: a left and a right turn of the same radius match
    assert look_ahead_m(FakeCourse(-2.0), 20.0) == pytest.approx(look_ahead_m(FakeCourse(2.0), 20.0))


def test_look_ahead_never_goes_below_the_minimum_on_a_very_sharp_curve() -> None:
    # 12 / (1 + 10) is about 1.09 m, below the 2 m minimum
    assert look_ahead_m(FakeCourse(10.0), 20.0) == pytest.approx(MIN_LOOK_AHEAD_M)

"""
Unit test of TrajectoryVerifier

Author: Khushi
"""

import numpy as np
import pytest
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/state")
sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/control/vfh")
from state import State
from trajectory_verifier import TrajectoryVerifier


class _FakeHistogram:
    """
    Minimal stand-in for PolarHistogram exposing only the getter
    TrajectoryVerifier actually reads
    """

    def __init__(self, density):
        self.density = np.array(density, dtype=float)

    def get_smoothed_density(self):
        return self.density


class _FakeMapper:
    """
    Minimal stand-in for a mapper exposing only get_histogram()
    """

    def __init__(self, density):
        self.histogram = _FakeHistogram(density)

    def get_histogram(self):
        return self.histogram

    def set_density(self, density):
        self.histogram.density = np.array(density, dtype=float)


def test_invalid_danger_density_raises():
    mapper = _FakeMapper([0.0])
    state = State(x_m=0.0, y_m=0.0)

    with pytest.raises(ValueError):
        TrajectoryVerifier(mapper, state, 0.0, 0.0, danger_density=0.0)
    with pytest.raises(ValueError):
        TrajectoryVerifier(mapper, state, 0.0, 0.0, danger_density=-1.0)


def test_invalid_goal_tolerance_raises():
    mapper = _FakeMapper([0.0])
    state = State(x_m=0.0, y_m=0.0)

    with pytest.raises(ValueError):
        TrajectoryVerifier(mapper, state, 0.0, 0.0, goal_tolerance_m=-1.0)


def test_distance_to_goal_is_euclidean():
    mapper = _FakeMapper([0.0])
    state = State(x_m=0.0, y_m=0.0)
    verifier = TrajectoryVerifier(mapper, state, target_x_m=3.0, target_y_m=4.0)

    assert verifier.distance_to_goal_m() == pytest.approx(5.0)


def test_reached_goal_within_tolerance():
    mapper = _FakeMapper([0.0])
    state = State(x_m=9.0, y_m=0.0)
    verifier = TrajectoryVerifier(mapper, state, target_x_m=10.0, target_y_m=0.0,
                                  goal_tolerance_m=2.0)

    assert verifier.reached_goal() is True


def test_not_reached_goal_outside_tolerance():
    mapper = _FakeMapper([0.0])
    state = State(x_m=0.0, y_m=0.0)
    verifier = TrajectoryVerifier(mapper, state, target_x_m=10.0, target_y_m=0.0,
                                  goal_tolerance_m=2.0)

    assert verifier.reached_goal() is False


def test_max_density_seen_tracks_running_maximum():
    mapper = _FakeMapper([0.1, 0.2])
    state = State(x_m=0.0, y_m=0.0)
    verifier = TrajectoryVerifier(mapper, state, 0.0, 0.0)

    verifier.update(0.1)
    assert verifier.get_max_density_seen() == pytest.approx(0.2)

    mapper.set_density([0.05])  # a lower reading afterwards must not lower the running max
    verifier.update(0.1)
    assert verifier.get_max_density_seen() == pytest.approx(0.2)

    mapper.set_density([0.9])
    verifier.update(0.1)
    assert verifier.get_max_density_seen() == pytest.approx(0.9)


def test_near_collision_count_increments_at_or_above_danger_density():
    mapper = _FakeMapper([0.5])
    state = State(x_m=0.0, y_m=0.0)
    verifier = TrajectoryVerifier(mapper, state, 0.0, 0.0, danger_density=1.0)

    verifier.update(0.1)
    assert verifier.get_near_collision_count() == 0

    mapper.set_density([1.0])
    verifier.update(0.1)
    assert verifier.get_near_collision_count() == 1

    mapper.set_density([2.0])
    verifier.update(0.1)
    assert verifier.get_near_collision_count() == 2


def test_draw_does_not_raise():
    mapper = _FakeMapper([0.0])
    state = State(x_m=0.0, y_m=0.0)
    verifier = TrajectoryVerifier(mapper, state, 0.0, 0.0)
    verifier.update(0.1)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots()
    elems = []

    verifier.draw(axes, elems)

    assert len(elems) == 1
    plt.close(figure)

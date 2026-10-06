"""
Unit test of PerformanceBenchmark

Author: Khushi
"""

import pytest
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/control/vfh")
from performance_benchmark import PerformanceBenchmark


class _FakeTimedComponent:
    """
    Minimal stand-in for PolarHistogramMapper/VfhController exposing only
    the getter PerformanceBenchmark actually reads
    """

    def __init__(self, duration_s=0.0):
        self.duration_s = duration_s

    def get_last_update_duration_s(self):
        return self.duration_s

    def set_duration_s(self, duration_s):
        self.duration_s = duration_s


def test_initial_stats_are_zero():
    benchmark = PerformanceBenchmark(_FakeTimedComponent(), _FakeTimedComponent())

    assert benchmark.get_last_duration_s() == 0.0
    assert benchmark.get_mean_duration_s() == 0.0
    assert benchmark.get_max_duration_s() == 0.0


def test_last_duration_is_mapper_plus_controller():
    mapper = _FakeTimedComponent(0.002)
    controller = _FakeTimedComponent(0.001)
    benchmark = PerformanceBenchmark(mapper, controller)

    benchmark.update(0.1)

    assert benchmark.get_last_duration_s() == pytest.approx(0.003)


def test_mean_duration_averages_across_updates():
    mapper = _FakeTimedComponent(0.0)
    controller = _FakeTimedComponent(0.0)
    benchmark = PerformanceBenchmark(mapper, controller)

    mapper.set_duration_s(0.001)
    controller.set_duration_s(0.001)
    benchmark.update(0.1)  # total 0.002

    mapper.set_duration_s(0.003)
    controller.set_duration_s(0.001)
    benchmark.update(0.1)  # total 0.004

    assert benchmark.get_mean_duration_s() == pytest.approx(0.003)


def test_max_duration_tracks_worst_case():
    mapper = _FakeTimedComponent(0.0)
    controller = _FakeTimedComponent(0.0)
    benchmark = PerformanceBenchmark(mapper, controller)

    mapper.set_duration_s(0.005)
    benchmark.update(0.1)
    assert benchmark.get_max_duration_s() == pytest.approx(0.005)

    mapper.set_duration_s(0.001)  # a smaller reading afterwards must not lower the max
    benchmark.update(0.1)
    assert benchmark.get_max_duration_s() == pytest.approx(0.005)

    mapper.set_duration_s(0.009)
    benchmark.update(0.1)
    assert benchmark.get_max_duration_s() == pytest.approx(0.009)


def test_draw_does_not_raise():
    benchmark = PerformanceBenchmark(_FakeTimedComponent(0.001), _FakeTimedComponent(0.001))
    benchmark.update(0.1)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots()
    elems = []

    benchmark.draw(axes, elems)

    assert len(elems) == 1
    plt.close(figure)

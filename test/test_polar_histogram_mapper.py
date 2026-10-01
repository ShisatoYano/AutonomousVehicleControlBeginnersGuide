"""
Unit test of PolarHistogramMapper

Author: Khushi
"""

import matplotlib as mpl
mpl.use("Agg")
import matplotlib.pyplot as plt
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/mapping/polar_histogram")
sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/state")
from polar_histogram_mapper import PolarHistogramMapper
from state import State


class DummyScanPoint:
    """
    Minimal stand-in for a LiDAR ScanPoint, exposing only what
    PolarHistogramMapper reads from it
    """

    def __init__(self, angle_rad, distance_m):
        self.angle_rad = angle_rad
        self._distance_m = distance_m

    def get_distance_m(self):
        return self._distance_m


def _new_axes():
    plt.clf()
    plt.close()
    figure = plt.figure(figsize=(8, 8))
    return figure.add_subplot(111)


def test_update_stores_vehicle_pose():
    mapper = PolarHistogramMapper(num_sectors=36, smoothing_window=3)
    state = State(x_m=1.0, y_m=2.0, yaw_rad=0.3)

    mapper.update([], state)

    assert mapper.vehicle_x_m == 1.0
    assert mapper.vehicle_y_m == 2.0
    assert mapper.vehicle_yaw_rad == 0.3


def test_update_and_draw_without_error():
    mapper = PolarHistogramMapper(num_sectors=36, smoothing_window=3)
    point_cloud = [DummyScanPoint(0.0, 3.0), DummyScanPoint(1.0, 5.0)]
    state = State(x_m=1.0, y_m=2.0, yaw_rad=0.3)

    mapper.update(point_cloud, state)

    axes = _new_axes()
    elems = []
    mapper.draw(axes, elems)

    assert len(elems) > 0


def test_draw_with_no_detections_adds_only_the_full_valley():
    # with no LiDAR detections the density ring(Step 1) contributes
    # nothing, but candidate valley detection(Step 2) still finds the
    # entire circle navigable and draws a single valley arc for it
    mapper = PolarHistogramMapper(num_sectors=36)
    state = State()

    mapper.update([], state)

    axes = _new_axes()
    elems = []
    mapper.draw(axes, elems)

    assert len(elems) == 1
    assert len(mapper.get_valley_detector().get_valleys()) == 1


def test_update_detects_valley_between_two_obstacle_clusters():
    mapper = PolarHistogramMapper(num_sectors=36, smoothing_window=1, valley_density_threshold=0.2)
    # two tight clusters of close obstacles roughly opposite each other,
    # leaving open gaps in between
    point_cloud = [DummyScanPoint(0.0, 1.0), DummyScanPoint(0.05, 1.0),
                  DummyScanPoint(3.14, 1.0), DummyScanPoint(3.09, 1.0)]
    state = State()

    mapper.update(point_cloud, state)

    valleys = mapper.get_valley_detector().get_valleys()
    assert len(valleys) >= 1


def test_get_valley_detector_returns_configured_threshold():
    mapper = PolarHistogramMapper(num_sectors=36, valley_density_threshold=0.4)

    assert mapper.get_valley_detector().get_density_threshold() == 0.4

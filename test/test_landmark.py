"""Tests for Landmark and LandmarkList."""

from pathlib import Path
import sys

import numpy as np

_components = str(Path(__file__).absolute().parent) + "/../src/components/landmark"
sys.path.append(_components)

from landmark import Landmark
from landmark_list import LandmarkList


def test_landmark_predicted_range_bearing_at():
    robot = np.array([[0.0], [0.0], [0.0]])
    z = Landmark.predicted_range_bearing_at(robot, 3.0, 4.0)
    assert abs(z[0, 0] - 5.0) < 1e-6
    assert abs(z[1, 0] - np.arctan2(4, 3)) < 1e-6


def test_landmark_instance_method():
    robot = np.array([[0.0], [0.0], [0.0]])
    lm = Landmark(3.0, 4.0)
    z = lm.predicted_range_bearing(robot)
    assert np.allclose(z, Landmark.predicted_range_bearing_at(robot, 3.0, 4.0))


def test_landmark_list_as_xy_tuples():
    ll = LandmarkList()
    ll.add_landmark(Landmark(1.0, 2.0))
    ll.add_landmark(Landmark(3.0, 4.0))
    assert ll.as_xy_tuples() == [(1.0, 2.0), (3.0, 4.0)]
    assert len(ll) == 2

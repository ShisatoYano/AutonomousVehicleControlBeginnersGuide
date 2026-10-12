"""
Tests for LandmarkRangeBearingSensor (range–bearing landmark observation).
"""

from pathlib import Path
import sys

import numpy as np

_components = str(Path(__file__).absolute().parent) + "/../src/components"
sys.path.append(_components + "/localization/ekf_slam")
sys.path.append(_components + "/sensors/landmark_range_bearing")
sys.path.append(_components + "/landmark")

from landmark import Landmark
from landmark_list import LandmarkList
from landmark_range_bearing_sensor import LandmarkRangeBearingSensor


def test_observation_model_matches_landmark():
    robot = np.array([[0.0], [0.0], [0.0], [1.0]])
    lm = Landmark(3.0, 4.0)
    z_sensor = LandmarkRangeBearingSensor.observation_model(robot, lm)
    z_lm = lm.predicted_range_bearing(robot)
    assert np.allclose(z_sensor, z_lm)


def test_observe_visible_landmarks_in_range():
    np.random.seed(0)
    robot = np.array([[0.0], [0.0], [0.0], [0.0]])
    ll = LandmarkList()
    ll.add_landmark(Landmark(5.0, 0.0))
    ll.add_landmark(Landmark(1000.0, 0.0))
    sensor_short = LandmarkRangeBearingSensor(
        sigma_r=0.01, sigma_phi=0.01, max_range_m=10.0
    )
    obs = sensor_short.observe_visible_landmarks(robot, ll)
    assert len(obs) == 1
    assert obs[0].shape == (2, 1)
    assert abs(obs[0][0, 0] - 5.0) < 0.5


def test_observe_noisy_shape():
    sensor = LandmarkRangeBearingSensor(sigma_r=0.2, sigma_phi=0.1, max_range_m=15.0)
    robot = np.array([[1.0], [2.0], [0.5], [1.0]])
    z = sensor.observe_noisy(robot, Landmark(4.0, 6.0))
    assert z.shape == (2, 1)
    assert np.isfinite(z).all()

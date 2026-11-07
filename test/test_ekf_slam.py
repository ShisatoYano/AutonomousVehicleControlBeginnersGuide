"""
Test of EKF-SLAM (Simultaneous Localization and Mapping) Simulation

Author: Contribution following HOWTOCONTRIBUTE.md
"""

from pathlib import Path
import sys
import numpy as np

ekf_slam_path = str(Path(__file__).absolute().parent) + "/../src/simulations/localization/ekf_slam"
sys.path.append(ekf_slam_path)
import ekf_slam
from state import State
from motion_model import jacobian_F, jacobian_G
from observation_model import build_H_matrix

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/landmark")
from landmark import Landmark
from data_association import mahalanobis_distance, nearest_neighbor_association
from landmark_manager import initialize_landmark_from_observation, augment_state


def test_simulation():
    ekf_slam.show_plot = False
    ekf_slam.main()


def test_motion_model_predict():
    robot = np.array([[0.0], [0.0], [0.0], [1.0]])
    u = np.array([[0.0], [0.1]])
    next_state = State.motion_model(robot, u, 0.1)
    assert next_state.shape == (4, 1)
    assert np.isfinite(next_state).all()


def test_motion_model_jacobians():
    robot = np.array([[1.0], [2.0], [0.5], [1.0]])
    u = np.array([[0.5], [0.05]])
    pred = State.motion_model(robot, u, 0.1)
    F = jacobian_F(pred, u, 0.1)
    G = jacobian_G(pred, 0.1)
    assert F.shape == (4, 4)
    assert G.shape == (4, 2)
    assert np.isfinite(F).all() and np.isfinite(G).all()


def test_landmark_predicted_range_bearing():
    robot = np.array([[0.0], [0.0], [0.0]])
    z = Landmark.predicted_range_bearing_at(robot, 3.0, 4.0)
    assert z.shape == (2, 1)
    assert abs(z[0, 0] - 5.0) < 1e-6
    assert abs(z[1, 0] - np.arctan2(4, 3)) < 1e-6


def test_observation_jacobian():
    robot = np.array([[0.0], [0.0], [0.0]])
    H = build_H_matrix(robot, 3.0, 4.0, 0, 6)
    assert H.shape == (2, 6)
    assert np.isfinite(H).all()


def test_data_association_mahalanobis():
    z1 = np.array([[1.0], [0.0]])
    z2 = np.array([[1.1], [0.05]])
    S = np.eye(2) * 0.01
    d = mahalanobis_distance(z1, z2, S)
    assert d >= 0 and np.isfinite(d)


def test_data_association_nearest_neighbor():
    obs = [np.array([[5.0], [0.1]])]
    pred = [np.array([[5.0], [0.0]])]
    S_list = [np.eye(2)]
    matches = nearest_neighbor_association(obs, pred, S_list, [0], 3.0)
    assert len(matches) == 1
    assert matches[0][1] == 0


def test_landmark_initialization():
    robot = np.array([[0.0], [0.0], [0.0]])
    z = np.array([[5.0], [np.arctan2(4, 3)]])
    lm, G_r, G_l = initialize_landmark_from_observation(robot, z, None, None)
    assert lm.shape == (2, 1)
    assert G_r.shape == (2, 4)
    assert abs(lm[0, 0] - 3.0) < 0.1 and abs(lm[1, 0] - 4.0) < 0.1


def test_augment_state():
    mu = np.array([[0.0], [0.0], [0.0], [0.0]])
    Sigma = np.eye(4) * 0.1
    z = np.array([[5.0], [0.0]])
    R = np.diag([0.1, 0.01])
    mu_new, Sigma_new = augment_state(mu, Sigma, z, mu[0:3], R)
    assert mu_new.shape[0] == 6
    assert Sigma_new.shape == (6, 6)
    assert np.allclose(Sigma_new, Sigma_new.T)

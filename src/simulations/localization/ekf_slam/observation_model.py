"""
observation_model.py

Range-bearing observation model and Jacobians for EKF-SLAM.
Measurement: range r, bearing phi (relative to robot yaw).
"""

import numpy as np
from math import atan2, sqrt


def observe_landmark(robot_state, lx, ly):
    """
    Predicted range and bearing from robot to landmark.
    robot_state: (3, 1) [x, y, yaw]
    lx, ly: landmark position (scalars)
    Returns: (2, 1) [range, bearing]
    """
    x, y, yaw = robot_state[0, 0], robot_state[1, 0], robot_state[2, 0]
    dx = lx - x
    dy = ly - y
    r = sqrt(dx * dx + dy * dy)
    if r < 1e-9:
        return np.array([[0.0], [0.0]])
    global_bearing = atan2(dy, dx)
    bearing = global_bearing - yaw
    # normalize bearing to [-pi, pi]
    while bearing > np.pi:
        bearing -= 2 * np.pi
    while bearing < -np.pi:
        bearing += 2 * np.pi
    return np.array([[r], [bearing]])


def jacobian_H(robot_state, lx, ly, landmark_index):
    """
    Jacobian of observation [range, bearing] w.r.t. state vector.
    State vector is [x, y, yaw, l1_x, l1_y, l2_x, l2_y, ...].
    landmark_index: 0-based index of this landmark (so its state indices are 3+2*j, 3+2*j+1).
    Returns: (2, state_dim) Jacobian H for this landmark (state_dim = 3 + 2*num_landmarks).
    """
    x, y, yaw = robot_state[0, 0], robot_state[1, 0], robot_state[2, 0]
    dx = lx - x
    dy = ly - y
    r_sq = dx * dx + dy * dy
    r = sqrt(r_sq)
    if r < 1e-9:
        r = 1e-9
        r_sq = 1e-18
    # dr/dx, dr/dy, dr/dyaw, dr/dlx, dr/dly
    dr_dx = -dx / r
    dr_dy = -dy / r
    dphi_dx = dy / r_sq
    dphi_dy = -dx / r_sq
    dphi_dyaw = -1.0
    dr_dlx = dx / r
    dr_dly = dy / r
    dphi_dlx = -dy / r_sq
    dphi_dly = dx / r_sq
    # We need to return H with full state dimension; caller will pass state_dim
    # So we don't know state_dim here. Return partial blocks and let caller build H.
    H_robot = np.array([[dr_dx, dr_dy, 0],
                        [dphi_dx, dphi_dy, dphi_dyaw]])
    H_landmark = np.array([[dr_dlx, dr_dly],
                           [dphi_dlx, dphi_dly]])
    return H_robot, H_landmark, landmark_index


def build_H_matrix(robot_state, lx, ly, landmark_index, state_dim):
    """
    Build full (2, state_dim) Jacobian H for one landmark.
    """
    H_robot, H_landmark, j = jacobian_H(robot_state, lx, ly, landmark_index)
    H = np.zeros((2, state_dim))
    H[:, 0:3] = H_robot
    H[:, 3 + 2 * j:3 + 2 * j + 2] = H_landmark
    return H


def simulate_observation(robot_state, lx, ly, sigma_r, sigma_phi):
    """
    Simulate noisy range-bearing measurement.
    robot_state: (3, 1), lx, ly: landmark position
    sigma_r: range noise std, sigma_phi: bearing noise std [rad]
    Returns: (2, 1) noisy [range, bearing]
    """
    z = observe_landmark(robot_state, lx, ly)
    z[0, 0] += sigma_r * np.random.randn()
    z[1, 0] += sigma_phi * np.random.randn()
    while z[1, 0] > np.pi:
        z[1, 0] -= 2 * np.pi
    while z[1, 0] < -np.pi:
        z[1, 0] += 2 * np.pi
    return z

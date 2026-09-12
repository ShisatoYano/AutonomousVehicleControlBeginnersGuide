"""
observation_model.py

EKF linearization only: Jacobians for range–bearing measurements.
Noise-free measurement prediction lives on Landmark; noisy simulation on LandmarkRangeBearingSensor.

Measurement: range r, bearing phi (relative to robot yaw).
"""

import numpy as np
from math import sqrt


def jacobian_H(robot_state, lx, ly, landmark_index):
    """
    Jacobian of observation [range, bearing] w.r.t. state vector.
    State vector is [x, y, yaw, speed, l1_x, l1_y, l2_x, l2_y, ...].
    landmark_index: 0-based index of this landmark (state indices 4+2*j, 4+2*j+1).
    Returns partial blocks; build_H_matrix adds zero columns for speed and places landmark block.
    """
    x, y, yaw = robot_state[0, 0], robot_state[1, 0], robot_state[2, 0]
    dx = lx - x
    dy = ly - y
    r_sq = dx * dx + dy * dy
    r = sqrt(r_sq)
    r_min_jacobian = 0.3
    if r < r_min_jacobian:
        r = r_min_jacobian
        r_sq = r * r
    elif r < 1e-9:
        r = 1e-9
        r_sq = 1e-18
    dr_dx = -dx / r
    dr_dy = -dy / r
    dphi_dx = dy / r_sq
    dphi_dy = -dx / r_sq
    dphi_dyaw = -1.0
    dr_dlx = dx / r
    dr_dly = dy / r
    dphi_dlx = -dy / r_sq
    dphi_dly = dx / r_sq
    H_robot = np.array([[dr_dx, dr_dy, 0],
                        [dphi_dx, dphi_dy, dphi_dyaw]])
    H_landmark = np.array([[dr_dlx, dr_dly],
                           [dphi_dlx, dphi_dly]])
    return H_robot, H_landmark, landmark_index


def build_H_matrix(robot_state, lx, ly, landmark_index, state_dim):
    """
    Build full (2, state_dim) Jacobian H for one landmark.
    Robot block is 4 columns: (x, y, yaw, speed); range/bearing do not depend on speed.
    """
    H_robot, H_landmark, j = jacobian_H(robot_state, lx, ly, landmark_index)
    H = np.zeros((2, state_dim))
    H[:, 0:3] = H_robot
    H[:, 3] = 0.0
    H[:, 4 + 2 * j:4 + 2 * j + 2] = H_landmark
    return H

"""
data_association.py

Nearest-neighbor data association with Mahalanobis distance gate.
"""

import numpy as np


def mahalanobis_distance(z_actual, z_predicted, S):
    """
    Mahalanobis distance: (z_actual - z_predicted)^T @ inv(S) @ (z_actual - z_predicted)
    z_actual, z_predicted: (2, 1) measurement vectors
    S: (2, 2) innovation covariance
    Returns: scalar distance
    """
    diff = z_actual - z_predicted
    # Normalize bearing difference to [-pi, pi]
    diff[1, 0] = _normalize_angle(diff[1, 0])
    try:
        s_diff = np.linalg.solve(S, diff)
        d_sq = (diff.T @ s_diff)[0, 0]
        return np.sqrt(max(0, d_sq))
    except np.linalg.LinAlgError:
        return np.inf


def _normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


# Landmark id returned when observation should not be added as new (potential duplicate)
POTENTIAL_DUPLICATE = -2


def nearest_neighbor_association(observations, predicted_observations, innovation_covs,
                                 landmark_ids, gate_threshold):
    """
    For each observation, find nearest landmark within Mahalanobis gate.
    observations: list of (2, 1) arrays - actual measurements
    predicted_observations: list of (2, 1) arrays - one per known landmark
    innovation_covs: list of (2, 2) arrays - S matrix per landmark
    landmark_ids: list of landmark indices (0-based)
    gate_threshold: max Mahalanobis distance to accept a match
    Returns: list of (obs_index, landmark_id).
      landmark_id >= 0: match to that landmark (state index).
      landmark_id == -1: truly new landmark (add to state).
      landmark_id == POTENTIAL_DUPLICATE (-2): within gate of an already-used landmark;
        do NOT add as new (duplicate-landmark guard).
    """
    num_obs = len(observations)
    num_landmarks = len(landmark_ids)
    matches = []
    used_landmarks = set()
    for o in range(num_obs):
        z = observations[o]
        best_landmark = -1
        best_dist = gate_threshold + 1e-6
        min_dist_any = np.inf  # min distance to any landmark (including used)
        for L in range(num_landmarks):
            d = mahalanobis_distance(z, predicted_observations[L], innovation_covs[L])
            min_dist_any = min(min_dist_any, d)
            if landmark_ids[L] in used_landmarks:
                continue
            if d < best_dist:
                best_dist = d
                best_landmark = landmark_ids[L]
        if best_landmark >= 0:
            used_landmarks.add(best_landmark)
            matches.append((o, best_landmark))
        elif min_dist_any <= gate_threshold:
            matches.append((o, POTENTIAL_DUPLICATE))
        else:
            matches.append((o, -1))  # truly new landmark
    return matches

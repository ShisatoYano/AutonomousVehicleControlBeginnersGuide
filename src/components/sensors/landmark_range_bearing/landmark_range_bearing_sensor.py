"""
landmark_range_bearing_sensor.py

Range–bearing landmark sensor (simulation), following the pattern of gnss.Gnss.
Noise-free prediction uses Landmark; EKF Jacobians remain in observation_model.
"""

import sys
from pathlib import Path

import numpy as np

_landmark_path = str(Path(__file__).absolute().parent.parent.parent) + "/landmark"
sys.path.append(_landmark_path)
from landmark import Landmark


class LandmarkRangeBearingSensor:
    """
    Simulates noisy range and bearing measurements to point landmarks in 2D.
    """

    def __init__(self, sigma_r=0.2, sigma_phi=0.1, max_range_m=15.0, color="c"):
        """
        sigma_r: range noise standard deviation [m]
        sigma_phi: bearing noise standard deviation [rad]
        max_range_m: landmarks beyond this predicted range are not observed
        color: optional draw color (reserved for future visualization)
        """
        self.sigma_r = sigma_r
        self.sigma_phi = sigma_phi
        self.max_range_m = max_range_m
        self.DRAW_COLOR = color
        self.NOISE_VAR_MAT = np.diag([sigma_r ** 2, sigma_phi ** 2])

    @staticmethod
    def observation_model(robot_state, landmark):
        """
        Noise-free predicted measurement [range, bearing] from robot pose to landmark.
        robot_state: (3, 1) or (4, 1); only x, y, yaw are used.
        landmark: Landmark instance
        """
        return landmark.predicted_range_bearing(robot_state)

    def observe_noisy(self, robot_state, landmark):
        """Noisy range–bearing sample for one landmark."""
        z = landmark.predicted_range_bearing(robot_state).copy()
        z[0, 0] = max(0.0, z[0, 0] + self.sigma_r * np.random.randn())
        z[1, 0] = (
            z[1, 0] + self.sigma_phi * np.random.randn() + np.pi
        ) % (2 * np.pi) - np.pi
        return z

    def observe_visible_landmarks(self, robot_state, landmarks):
        """
        For each Landmark, if predicted range <= max_range_m, append one noisy observation.
        landmarks: LandmarkList or iterable of Landmark
        Returns: list of (2, 1) arrays in iteration order (only in-range).
        """
        if hasattr(landmarks, "get_list"):
            iterable = landmarks.get_list()
        else:
            iterable = landmarks
        observations = []
        for lm in iterable:
            z_pred = lm.predicted_range_bearing(robot_state)
            if z_pred[0, 0] <= self.max_range_m:
                observations.append(self.observe_noisy(robot_state, lm))
        return observations

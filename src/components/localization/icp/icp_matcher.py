"""
icp_matcher.py

Author: Yuvraj Gupta

Reference:
P. J. Besl and N. D. McKay, "A Method for Registration of 3-D Shapes", IEEE TPAMI, 1992
K. S. Arun, T. S. Huang and S. D. Blostein, "Least-Squares Fitting of Two 3-D Point Sets", IEEE TPAMI, 1987
"""

import numpy as np
import numpy.typing as npt
from scipy.spatial import cKDTree


class IcpMatcher:
    """
    2D point-to-point ICP (Iterative Closest Point)
    Finds the rotation R and translation t which move the source points onto the target points.
    Each iteration:
    1. pair every source point with its nearest target point (kd-tree)
    2. drop pairs that are too far apart (outliers)
    3. solve the best R, t for the pairs in closed form with SVD
    4. move the source points and repeat until the change is small
    """

    def __init__(self, max_iteration: int = 50, tolerance: float = 1e-5,
                 max_pair_distance_m: float = 2.0):
        """
        Constructor
        max_iteration: Maximum number of iterations
        tolerance: Stop when one iteration moves the points less than this value (translation[m] + rotation[rad])
        max_pair_distance_m: Pairs farther than this distance are treated as outliers[m]
        """

        self.MAX_ITERATION = max_iteration
        self.TOLERANCE = tolerance
        self.MAX_PAIR_DISTANCE_M = max_pair_distance_m

        self.mean_error_m = 0.0
        self.iteration_num = 0

    @staticmethod
    def best_fit_transform(source: npt.NDArray[np.float64],
                           target: npt.NDArray[np.float64]) -> tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:
        """
        Function to calculate R, t minimizing sum |R p_i + t - q_i|^2 for paired points
        source: Paired source points p_i, shape (N, 2)
        target: Paired target points q_i, shape (N, 2)
        Return: R shape (2, 2), t shape (2,)
        """

        # 1. move both point sets to their centroids
        source_centroid = np.mean(source, axis=0)
        target_centroid = np.mean(target, axis=0)
        source_centered = source - source_centroid
        target_centered = target - target_centroid

        # 2. the rotation comes from the SVD of the cross covariance matrix
        cross_cov = source_centered.T @ target_centered
        U, _, Vt = np.linalg.svd(cross_cov)
        R = Vt.T @ U.T

        # avoid a reflection (det = -1) instead of a rotation
        if np.linalg.det(R) < 0:
            Vt[1, :] *= -1
            R = Vt.T @ U.T

        # 3. the translation moves the rotated source centroid onto the target centroid
        t = target_centroid - R @ source_centroid
        return R, t

    def match(self, source: npt.NDArray[np.float64], target: npt.NDArray[np.float64],
              init_R: npt.NDArray[np.float64] | None = None,
              init_t: npt.NDArray[np.float64] | None = None) -> tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:
        """
        Function to align the source points to the target points
        source: Source points, shape (N, 2)
        target: Target points, shape (M, 2)
        init_R: Initial guess of rotation, shape (2, 2)
        init_t: Initial guess of translation, shape (2,)
        Return: Total R shape (2, 2), total t shape (2,), so that target ~ R source + t
        """

        total_R = np.eye(2) if init_R is None else init_R.copy()
        total_t = np.zeros(2) if init_t is None else init_t.copy()
        if len(source) < 3 or len(target) < 3: return total_R, total_t

        tree = cKDTree(target)
        moved = source @ total_R.T + total_t

        for i in range(self.MAX_ITERATION):
            distances, indices = tree.query(moved)
            inlier = distances < self.MAX_PAIR_DISTANCE_M
            if np.count_nonzero(inlier) < 3: break

            R, t = self.best_fit_transform(moved[inlier], target[indices[inlier]])
            moved = moved @ R.T + t
            total_R = R @ total_R
            total_t = R @ total_t + t

            self.mean_error_m = float(np.mean(distances[inlier]))
            self.iteration_num = i + 1

            # stop when this iteration moved the points only a little
            step_m = np.hypot(t[0], t[1]) + abs(np.arctan2(R[1, 0], R[0, 0]))
            if step_m < self.TOLERANCE: break

        return total_R, total_t

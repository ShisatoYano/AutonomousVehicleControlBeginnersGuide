"""
Test of 2D ICP (Iterative Closest Point) matcher

Author: Yuvraj Gupta
"""

from pathlib import Path
import sys
import numpy as np
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/localization/icp")
from icp_matcher import IcpMatcher


def rotation(yaw_rad: float) -> np.ndarray:
    return np.array([[np.cos(yaw_rad), -np.sin(yaw_rad)],
                     [np.sin(yaw_rad), np.cos(yaw_rad)]])


def l_shape_points() -> np.ndarray:
    wall_x = np.stack([np.linspace(0.0, 10.0, 50), np.zeros(50)], axis=1)
    wall_y = np.stack([np.zeros(30), np.linspace(0.5, 6.0, 30)], axis=1)
    return np.vstack([wall_x, wall_y])


def test_best_fit_transform_recovers_exact_motion():
    source = l_shape_points()
    true_R, true_t = rotation(np.deg2rad(10.0)), np.array([0.5, -0.3])
    target = source @ true_R.T + true_t

    R, t = IcpMatcher.best_fit_transform(source, target)

    assert np.allclose(R, true_R)
    assert np.allclose(t, true_t)


def test_match_aligns_unpaired_points():
    np.random.seed(0)
    target = np.random.uniform(-10.0, 10.0, (200, 2))
    true_R, true_t = rotation(np.deg2rad(5.0)), np.array([0.3, 0.2])
    # source is the target seen from a moved sensor: target = R source + t
    source = (target - true_t) @ true_R

    R, t = IcpMatcher().match(source, target)

    assert np.allclose(R, true_R, atol=1e-3)
    assert np.allclose(t, true_t, atol=1e-2)

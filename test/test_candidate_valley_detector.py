"""
Unit test of CandidateValleyDetector

Author: Khushi
"""

import numpy as np
import pytest
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/mapping/polar_histogram")
from polar_histogram import PolarHistogram
from candidate_valley_detector import CandidateValleyDetector


def _histogram_with_density(density_values, max_range_m=10.0):
    """
    Helper to build a PolarHistogram and force its smoothed density to a
    known array, bypassing LiDAR-based accumulation so valley detection
    can be tested against exact, hand-picked density patterns
    """

    hist = PolarHistogram(num_sectors=len(density_values), max_range_m=max_range_m, smoothing_window=1)
    hist.smoothed_density = np.array(density_values, dtype=float)
    return hist


def test_invalid_threshold_raises():
    with pytest.raises(ValueError):
        CandidateValleyDetector(density_threshold=-0.1)


def test_no_obstacles_gives_one_full_valley():
    hist = _histogram_with_density([0.0] * 8)
    detector = CandidateValleyDetector(density_threshold=0.2)

    valleys = detector.detect(hist)

    assert len(valleys) == 1
    assert valleys[0].get_start_index() == 0
    assert valleys[0].get_end_index() == 7
    assert valleys[0].get_width_sectors() == 8


def test_all_sectors_blocked_gives_no_valleys():
    hist = _histogram_with_density([1.0] * 8)
    detector = CandidateValleyDetector(density_threshold=0.2)

    valleys = detector.detect(hist)

    assert valleys == []


def test_single_obstacle_leaves_one_large_valley():
    # sector 0 blocked, sectors 1-7 navigable
    hist = _histogram_with_density([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    detector = CandidateValleyDetector(density_threshold=0.2)

    valleys = detector.detect(hist)

    assert len(valleys) == 1
    assert valleys[0].get_start_index() == 1
    assert valleys[0].get_end_index() == 7
    assert valleys[0].get_width_sectors() == 7


def test_multiple_obstacles_create_multiple_valleys():
    # sectors 0 and 4 blocked out of 8: two valleys of width 3 each
    hist = _histogram_with_density([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    detector = CandidateValleyDetector(density_threshold=0.2)

    valleys = detector.detect(hist)

    assert len(valleys) == 2
    widths = sorted(v.get_width_sectors() for v in valleys)
    assert widths == [3, 3]


def test_valley_wraps_across_zero_index():
    # sectors 2 and 3 blocked out of 8: the navigable run is 4,5,6,7,0,1,
    # wrapping across the 0[deg] boundary
    hist = _histogram_with_density([0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0])
    detector = CandidateValleyDetector(density_threshold=0.2)

    valleys = detector.detect(hist)

    assert len(valleys) == 1
    assert valleys[0].get_start_index() == 4
    assert valleys[0].get_end_index() == 1
    assert valleys[0].get_width_sectors() == 6


def test_valley_center_angle():
    # sectors 0-3 blocked, 4-7 navigable(8 sectors -> 45[deg] each): the
    # valley's center should land on the 4/7 boundary sector split, 270[deg]
    hist = _histogram_with_density([1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0])
    detector = CandidateValleyDetector(density_threshold=0.2)

    valleys = detector.detect(hist)

    assert len(valleys) == 1
    assert np.isclose(valleys[0].get_center_angle_rad(), np.deg2rad(-90))


def test_threshold_is_configurable():
    hist = _histogram_with_density([0.1, 0.3, 0.1, 0.3])

    strict = CandidateValleyDetector(density_threshold=0.05)
    assert strict.detect(hist) == []

    lenient = CandidateValleyDetector(density_threshold=0.3)
    valleys = lenient.detect(hist)
    assert len(valleys) == 1
    assert valleys[0].get_width_sectors() == 4


def test_get_valleys_returns_last_detection():
    hist = _histogram_with_density([1.0, 0.0])
    detector = CandidateValleyDetector(density_threshold=0.2)

    assert detector.get_valleys() == []
    detector.detect(hist)
    assert len(detector.get_valleys()) == 1


def test_get_density_threshold():
    detector = CandidateValleyDetector(density_threshold=0.4)

    assert detector.get_density_threshold() == 0.4

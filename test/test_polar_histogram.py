"""
Unit test of PolarHistogram

Author: Khushi
"""

import numpy as np
import pytest
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/mapping/polar_histogram")
from polar_histogram import PolarHistogram


def test_initialize():
    hist = PolarHistogram(num_sectors=8, max_range_m=10.0, smoothing_window=1)

    assert hist.get_num_sectors() == 8
    assert np.isclose(hist.get_sector_angle_rad(), np.pi / 4.0)
    assert np.all(hist.get_raw_density() == 0.0)
    assert np.all(hist.get_smoothed_density() == 0.0)


def test_invalid_num_sectors_raises():
    with pytest.raises(ValueError):
        PolarHistogram(num_sectors=0)


def test_invalid_smoothing_window_raises():
    with pytest.raises(ValueError):
        PolarHistogram(smoothing_window=0)


def test_invalid_max_range_raises():
    with pytest.raises(ValueError):
        PolarHistogram(max_range_m=0.0)


def test_angle_to_sector_index():
    hist = PolarHistogram(num_sectors=4, max_range_m=10.0)

    # sector 0: [0, 90)deg, sector 1: [90, 180)deg,
    # sector 2: [180, 270)deg, sector 3: [270, 360)deg
    assert hist.angle_to_sector_index(0.0) == 0
    assert hist.angle_to_sector_index(np.deg2rad(91)) == 1
    assert hist.angle_to_sector_index(np.deg2rad(-91)) == 2  # -91deg == 269deg
    assert hist.angle_to_sector_index(np.deg2rad(-1)) == 3   # -1deg == 359deg


def test_sector_center_angle_rad_is_normalized():
    hist = PolarHistogram(num_sectors=4, max_range_m=10.0)

    # sector 2 covers [180, 270)deg, its center(225deg) should normalize to -135deg
    assert np.isclose(hist.sector_center_angle_rad(2), np.deg2rad(-135))


def test_update_places_obstacle_in_expected_sector():
    hist = PolarHistogram(num_sectors=4, max_range_m=10.0, smoothing_window=1)

    # single obstacle straight ahead(0[rad]), 2[m] away
    hist.update([0.0], [2.0])
    density = hist.get_raw_density()

    assert density[0] > 0.0
    assert np.all(density[1:] == 0.0)


def test_closer_obstacle_has_higher_density():
    hist = PolarHistogram(num_sectors=4, max_range_m=10.0, smoothing_window=1)

    hist.update([0.0, np.pi], [1.0, 9.0])
    density = hist.get_raw_density()

    assert density[0] > density[2]


def test_density_is_zero_beyond_max_range():
    hist = PolarHistogram(num_sectors=4, max_range_m=10.0, smoothing_window=1)

    hist.update([0.0], [15.0])

    assert np.all(hist.get_raw_density() == 0.0)


def test_smoothing_spreads_density_to_neighbour_sectors():
    hist = PolarHistogram(num_sectors=8, max_range_m=10.0, smoothing_window=2)

    hist.update([0.0], [2.0])
    smoothed = hist.get_smoothed_density()

    # immediate neighbour sectors receive some smoothed density
    assert smoothed[1] > 0.0
    assert smoothed[-1] > 0.0
    # a sector far away from the obstacle stays unaffected
    assert smoothed[4] == 0.0


def test_smoothing_disabled_when_window_is_one():
    hist = PolarHistogram(num_sectors=8, max_range_m=10.0, smoothing_window=1)

    hist.update([0.0], [2.0])

    assert np.array_equal(hist.get_smoothed_density(), hist.get_raw_density())

"""
polar_histogram.py

Author: Khushi
"""

import sys
from pathlib import Path

import numpy as np

sys.path.append(str(Path(__file__).absolute().parent) + "/../../common")
from angle_lib import pi_to_pi


class PolarHistogram:
    """
    1D polar obstacle density histogram class

    Divides the 360[deg] area around the vehicle into a configurable
    number of angular sectors and accumulates an obstacle density value
    in each sector from range sensor(LiDAR) measurements. This is the
    polar histogram construction step of the Vector Field Histogram(VFH)
    algorithm proposed by Borenstein and Koren(1991).

    Each measurement's magnitude is calculated by the following equation,
    which decreases linearly from a_gain at distance 0[m] to 0 at
    max_range_m, same as the original VFH paper:
        magnitude = max(a_gain - b_gain * distance_m, 0.0)
        b_gain = a_gain / max_range_m

    After accumulating raw magnitudes into sectors, a triangular moving
    average filter is applied between neighboring sectors around the
    circle to smooth out noise caused by sensor uncertainty.
    """

    def __init__(self, num_sectors=72, max_range_m=40.0, smoothing_window=5, a_gain=1.0):
        """
        Constructor
        num_sectors: Number of angular sectors dividing 360[deg](resolution)
        max_range_m: Max sensing range[m] used to normalize obstacle magnitude
        smoothing_window: Half-width of the triangular smoothing filter(sectors). 1 disables smoothing
        a_gain: Magnitude value at distance 0[m]
        """

        if num_sectors < 1:
            raise ValueError("num_sectors must be a positive integer")
        if smoothing_window < 1:
            raise ValueError("smoothing_window must be 1 or greater")
        if max_range_m <= 0.0:
            raise ValueError("max_range_m must be greater than 0")

        self.num_sectors = int(num_sectors)
        self.max_range_m = float(max_range_m)
        self.smoothing_window = int(smoothing_window)
        self.a_gain = float(a_gain)
        self.b_gain = self.a_gain / self.max_range_m
        self.sector_angle_rad = (2.0 * np.pi) / self.num_sectors

        self.raw_density = np.zeros(self.num_sectors)
        self.smoothed_density = np.zeros(self.num_sectors)

    def angle_to_sector_index(self, angle_rad):
        """
        Function to convert an angle[rad] to its sector index
        angle_rad: Angle relative to vehicle heading[rad], any range
        """

        angle_0_2pi = angle_rad % (2.0 * np.pi)
        index = int(np.floor(angle_0_2pi / self.sector_angle_rad)) % self.num_sectors
        return index

    def sector_center_angle_rad(self, index):
        """
        Function to get a sector's center angle, normalized between -pi and pi[rad]
        index: Sector index
        """

        center_angle_rad = (index + 0.5) * self.sector_angle_rad
        return pi_to_pi(center_angle_rad)

    def _magnitude(self, distance_m):
        """
        Private function to calculate a single measurement's obstacle magnitude
        distance_m: Sensed obstacle distance[m]
        """

        return max(self.a_gain - self.b_gain * distance_m, 0.0)

    def _accumulate(self, angle_list, distance_list):
        """
        Private function to accumulate raw obstacle density into sectors
        angle_list: List of sensed angle[rad], relative to vehicle heading
        distance_list: List of sensed distance[m], same length as angle_list
        """

        density = np.zeros(self.num_sectors)

        for angle_rad, distance_m in zip(angle_list, distance_list):
            if distance_m is None or distance_m <= 0.0 or distance_m > self.max_range_m:
                continue

            index = self.angle_to_sector_index(angle_rad)
            density[index] += self._magnitude(distance_m)

        return density

    def _smooth(self, density):
        """
        Private function to apply a triangular moving average filter
        between neighboring sectors around the circle
        density: Raw density array, length num_sectors
        """

        window = self.smoothing_window
        if window <= 1 or self.num_sectors <= 1:
            return density.copy()

        offsets = range(-(window - 1), window)
        weights = np.array([window - abs(j) for j in offsets], dtype=float)
        weights /= weights.sum()

        smoothed = np.zeros(self.num_sectors)
        for offset, weight in zip(offsets, weights):
            smoothed += weight * np.roll(density, -offset)

        return smoothed

    def update(self, angle_list, distance_list):
        """
        Function to update the polar histogram from new sensor measurements
        angle_list: List of sensed angle[rad], relative to vehicle heading
        distance_list: List of sensed distance[m], same length as angle_list
        """

        self.raw_density = self._accumulate(angle_list, distance_list)
        self.smoothed_density = self._smooth(self.raw_density)

    def get_num_sectors(self):
        """
        Function to get number of sectors
        """

        return self.num_sectors

    def get_sector_angle_rad(self):
        """
        Function to get a single sector's angular width[rad]
        """

        return self.sector_angle_rad

    def get_raw_density(self):
        """
        Function to get the raw(unsmoothed) density array
        """

        return self.raw_density

    def get_smoothed_density(self):
        """
        Function to get the smoothed density array
        """

        return self.smoothed_density

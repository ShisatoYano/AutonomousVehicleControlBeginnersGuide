"""
candidate_valley_detector.py

Author: Khushi
"""

import numpy as np


class Valley:
    """
    Represents one contiguous run of low-density polar histogram sectors,
    i.e. a candidate direction range the vehicle could steer into.

    Sector indices are circular(sector num_sectors-1 is adjacent to sector 0),
    so a valley may wrap around the 0[deg] boundary.
    """

    def __init__(self, start_index, end_index, width_sectors, center_angle_rad):
        """
        Constructor
        start_index: Sector index where the valley begins
        end_index: Sector index where the valley ends(inclusive)
        width_sectors: Number of sectors spanned by the valley
        center_angle_rad: Valley's center angle, normalized between -pi and pi[rad]
        """

        self.start_index = start_index
        self.end_index = end_index
        self.width_sectors = width_sectors
        self.center_angle_rad = center_angle_rad

    def get_start_index(self):
        """
        Function to get the sector index where the valley begins
        """

        return self.start_index

    def get_end_index(self):
        """
        Function to get the sector index where the valley ends(inclusive)
        """

        return self.end_index

    def get_width_sectors(self):
        """
        Function to get the number of sectors spanned by the valley
        """

        return self.width_sectors

    def get_center_angle_rad(self):
        """
        Function to get the valley's center angle[rad], normalized between -pi and pi
        """

        return self.center_angle_rad


class CandidateValleyDetector:
    """
    Candidate valley detection class

    This is the second step of the Vector Field Histogram(VFH) algorithm.
    Sectors whose smoothed obstacle density falls at or below a configurable
    threshold are considered navigable(free enough to steer into). Runs of
    consecutive navigable sectors around the circle are grouped into
    "valleys", each representing one candidate direction range for the next
    step(direction selection).
    """

    def __init__(self, density_threshold=0.2):
        """
        Constructor
        density_threshold: Sectors at or below this smoothed density value
                            are considered navigable. Must be 0 or greater
        """

        if density_threshold < 0.0:
            raise ValueError("density_threshold must be 0 or greater")

        self.density_threshold = float(density_threshold)
        self.valleys = []

    def detect(self, polar_histogram):
        """
        Function to detect candidate valleys from a PolarHistogram instance
        polar_histogram: PolarHistogram instance already updated this frame
        """

        density = polar_histogram.get_smoothed_density()
        num_sectors = polar_histogram.get_num_sectors()
        navigable = density <= self.density_threshold

        self.valleys = self._find_circular_valleys(navigable, num_sectors, polar_histogram)
        return self.valleys

    def _find_circular_valleys(self, navigable, num_sectors, polar_histogram):
        """
        Private function to group a circular boolean array of navigable
        sectors into contiguous Valley instances, handling wrap-around
        across the 0[deg] boundary
        navigable: Boolean array, True where a sector is navigable
        num_sectors: Number of sectors(length of navigable)
        polar_histogram: Used to look up each valley's center angle
        """

        if not np.any(navigable):
            return []

        if np.all(navigable):
            return [self._make_valley(0, num_sectors - 1, num_sectors, polar_histogram)]

        # rotate the array so it always starts just after a blocked sector.
        # At least one blocked sector exists here, since navigable isn't
        # all True, so every run in the rotated array is guaranteed to end
        # before the array wraps - no run can straddle its boundary
        blocked_indices = np.where(~navigable)[0]
        anchor = int(blocked_indices[0])
        rotated = np.roll(navigable, -(anchor + 1))

        valleys = []
        start = None
        for i in range(num_sectors):
            if rotated[i]:
                if start is None:
                    start = i
            elif start is not None:
                orig_start = (start + anchor + 1) % num_sectors
                orig_end = (i - 1 + anchor + 1) % num_sectors
                valleys.append(self._make_valley(orig_start, orig_end, num_sectors, polar_histogram))
                start = None

        return valleys

    def _make_valley(self, start_index, end_index, num_sectors, polar_histogram):
        """
        Private function to build a Valley instance from a start/end sector
        index pair, computing its width and center angle
        start_index: Sector index where the valley begins
        end_index: Sector index where the valley ends(inclusive)
        num_sectors: Number of sectors, used to compute circular width
        polar_histogram: Used to look up the center angle
        """

        width_sectors = (end_index - start_index) % num_sectors + 1
        mid_index = start_index + (width_sectors - 1) / 2.0
        center_angle_rad = polar_histogram.sector_center_angle_rad(mid_index)

        return Valley(start_index, end_index, width_sectors, center_angle_rad)

    def get_density_threshold(self):
        """
        Function to get the configured density threshold
        """

        return self.density_threshold

    def get_valleys(self):
        """
        Function to get the most recently detected list of Valley instances
        """

        return self.valleys

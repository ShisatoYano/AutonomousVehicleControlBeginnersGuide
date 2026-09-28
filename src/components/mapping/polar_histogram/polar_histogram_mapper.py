"""
polar_histogram_mapper.py

Author: Khushi
"""

import numpy as np
import matplotlib.patches as patches
from matplotlib.collections import PatchCollection

from polar_histogram import PolarHistogram


class PolarHistogramMapper:
    """
    Mapper class to build and visualize a polar obstacle density histogram
    around the vehicle from LiDAR point cloud data, following the mapping
    stage of the Vector Field Histogram(VFH) algorithm.

    Each frame, the histogram is drawn as a ring of colored wedges centered
    on the vehicle: denser(more blocked) sectors are drawn longer and closer
    to red, sparser(more open) sectors shorter and closer to green. Sector
    length and color are both normalized by the current frame's maximum
    density, so the ring stays readable regardless of how many obstacles
    are in range.

    Note on drawing accuracy: each measurement's angle/distance is computed
    from the LiDAR's actual mounted position(see OmniDirectionalLidar), so
    the histogram's underlying density data is geometrically accurate. The
    ring of wedges below is drawn centered on the vehicle's origin rather
    than the LiDAR's position, purely to keep the drawing code simple. Since
    the LiDAR is mounted ahead of the vehicle's origin, this is a visible
    approximation for very close obstacles(a few meters), shifting where
    they appear to sit within the ring by up to roughly 20[deg]. It does not
    affect the histogram data itself, or the valleys/direction that later
    steps derive from it - only where this ring is drawn on the plot.
    """

    def __init__(self, sensor_params=None, num_sectors=72, smoothing_window=5, ring_radius_m=8.0):
        """
        Constructor
        sensor_params: LiDAR's SensorParameters object, used for max sensing range
        num_sectors: Number of angular sectors dividing 360[deg](resolution)
        smoothing_window: Half-width of the triangular smoothing filter(sectors)
        ring_radius_m: Drawing radius of the densest sector's wedge on the global plot[m]
        """

        max_range_m = sensor_params.MAX_RANGE_M if sensor_params else 40.0

        self.histogram = PolarHistogram(num_sectors=num_sectors,
                                        max_range_m=max_range_m,
                                        smoothing_window=smoothing_window)
        self.ring_radius_m = ring_radius_m

        self.vehicle_x_m = 0.0
        self.vehicle_y_m = 0.0
        self.vehicle_yaw_rad = 0.0

    def update(self, point_cloud, state):
        """
        Function to update the polar histogram from the latest LiDAR point cloud
        point_cloud: List of ScanPoint objects from LiDAR. Each point's angle
                     is expected to be relative to the vehicle's heading
        state: Vehicle's state object
        """

        angle_list = [point.angle_rad for point in point_cloud]
        distance_list = [point.get_distance_m() for point in point_cloud]
        self.histogram.update(angle_list, distance_list)

        self.vehicle_x_m = state.get_x_m()
        self.vehicle_y_m = state.get_y_m()
        self.vehicle_yaw_rad = state.get_yaw_rad()

    def draw(self, axes, elems):
        """
        Function to draw the polar histogram as a ring of colored wedges around the vehicle
        axes: Axes object of figure
        elems: List of plot objects
        """

        density = self.histogram.get_smoothed_density()
        max_value = np.max(density)
        if max_value <= 0.0:
            return

        num_sectors = self.histogram.get_num_sectors()
        sector_deg = np.rad2deg(self.histogram.get_sector_angle_rad())
        yaw_deg = np.rad2deg(self.vehicle_yaw_rad)

        wedges = []
        for index in range(num_sectors):
            normalized = density[index] / max_value
            if normalized <= 0.0:
                continue

            center_deg = np.rad2deg(self.histogram.sector_center_angle_rad(index)) + yaw_deg
            theta_1 = center_deg - sector_deg / 2.0
            theta_2 = center_deg + sector_deg / 2.0
            radius_m = self.ring_radius_m * normalized

            wedges.append(patches.Wedge((self.vehicle_x_m, self.vehicle_y_m), radius_m,
                                        theta_1, theta_2,
                                        color=(normalized, 1.0 - normalized, 0.0), alpha=0.6))

        # Collecting every sector's wedge into a single PatchCollection and
        # adding it with one axes.add_collection() call is noticeably faster
        # than one axes.add_patch() call per wedge, which adds up at typical
        # sector counts(dozens of add_patch() calls every frame otherwise).
        # match_original=True keeps each wedge's own color/alpha instead of
        # applying one shared style to the whole collection
        collection = PatchCollection(wedges, match_original=True)
        axes.add_collection(collection)
        elems.append(collection)

    def get_histogram(self):
        """
        Function to get the underlying PolarHistogram instance
        """

        return self.histogram

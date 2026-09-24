"""
polar_histogram_mapper.py

Author: Khushi
"""

import numpy as np
import matplotlib.patches as patches

from polar_histogram import PolarHistogram
from candidate_valley_detector import CandidateValleyDetector


class PolarHistogramMapper:
    """
    Mapper class to build and visualize a polar obstacle density histogram
    around the vehicle from LiDAR point cloud data, following the mapping
    and candidate valley detection stages of the Vector Field Histogram(VFH)
    algorithm.

    Each frame, the histogram is drawn as a ring of colored wedges centered
    on the vehicle: denser(more blocked) sectors are drawn longer and closer
    to red, sparser(more open) sectors shorter and closer to green. Sector
    length and color are both normalized by the current frame's maximum
    density, so the ring stays readable regardless of how many obstacles
    are in range. Detected candidate valleys(navigable direction ranges,
    Step 2) are drawn as green arcs just outside that ring.
    """

    def __init__(self, sensor_params=None, num_sectors=72, smoothing_window=5,
                ring_radius_m=8.0, valley_density_threshold=0.2):
        """
        Constructor
        sensor_params: LiDAR's SensorParameters object, used for max sensing range
        num_sectors: Number of angular sectors dividing 360[deg](resolution)
        smoothing_window: Half-width of the triangular smoothing filter(sectors)
        ring_radius_m: Drawing radius of the densest sector's wedge on the global plot[m]
        valley_density_threshold: Smoothed density at/below which a sector counts as navigable
        """

        max_range_m = sensor_params.MAX_RANGE_M if sensor_params else 40.0

        self.histogram = PolarHistogram(num_sectors=num_sectors,
                                        max_range_m=max_range_m,
                                        smoothing_window=smoothing_window)
        self.valley_detector = CandidateValleyDetector(density_threshold=valley_density_threshold)
        self.ring_radius_m = ring_radius_m

        self.vehicle_x_m = 0.0
        self.vehicle_y_m = 0.0
        self.vehicle_yaw_rad = 0.0

    def update(self, point_cloud, state):
        """
        Function to update the polar histogram and candidate valleys from
        the latest LiDAR point cloud
        point_cloud: List of ScanPoint objects from LiDAR. Each point's angle
                     is expected to be relative to the vehicle's heading
        state: Vehicle's state object
        """

        angle_list = [point.angle_rad for point in point_cloud]
        distance_list = [point.get_distance_m() for point in point_cloud]
        self.histogram.update(angle_list, distance_list)
        self.valley_detector.detect(self.histogram)

        self.vehicle_x_m = state.get_x_m()
        self.vehicle_y_m = state.get_y_m()
        self.vehicle_yaw_rad = state.get_yaw_rad()

    def draw(self, axes, elems):
        """
        Function to draw the polar histogram as a ring of colored wedges
        around the vehicle, plus the detected candidate valleys as green
        arcs just outside that ring
        axes: Axes object of figure
        elems: List of plot objects
        """

        self._draw_density_ring(axes, elems)
        self._draw_valleys(axes, elems)

    def _draw_density_ring(self, axes, elems):
        """
        Private function to draw the polar histogram as a ring of colored wedges
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

        for index in range(num_sectors):
            normalized = density[index] / max_value
            if normalized <= 0.0:
                continue

            center_deg = np.rad2deg(self.histogram.sector_center_angle_rad(index)) + yaw_deg
            theta_1 = center_deg - sector_deg / 2.0
            theta_2 = center_deg + sector_deg / 2.0
            radius_m = self.ring_radius_m * normalized

            wedge = patches.Wedge((self.vehicle_x_m, self.vehicle_y_m), radius_m,
                                  theta_1, theta_2,
                                  color=(normalized, 1.0 - normalized, 0.0), alpha=0.6)
            axes.add_patch(wedge)
            elems.append(wedge)

    def _draw_valleys(self, axes, elems):
        """
        Private function to draw each detected candidate valley as a green
        arc-shaped wedge just outside the density ring
        axes: Axes object of figure
        elems: List of plot objects
        """

        sector_deg = np.rad2deg(self.histogram.get_sector_angle_rad())
        yaw_deg = np.rad2deg(self.vehicle_yaw_rad)
        inner_radius_m = self.ring_radius_m * 1.05
        outer_radius_m = self.ring_radius_m * 1.15

        for valley in self.valley_detector.get_valleys():
            width_deg = valley.get_width_sectors() * sector_deg
            center_deg = np.rad2deg(valley.get_center_angle_rad()) + yaw_deg
            theta_1 = center_deg - width_deg / 2.0
            theta_2 = center_deg + width_deg / 2.0

            valley_wedge = patches.Wedge((self.vehicle_x_m, self.vehicle_y_m), outer_radius_m,
                                         theta_1, theta_2, width=outer_radius_m - inner_radius_m,
                                         color="limegreen", alpha=0.5)
            axes.add_patch(valley_wedge)
            elems.append(valley_wedge)

    def get_histogram(self):
        """
        Function to get the underlying PolarHistogram instance
        """

        return self.histogram

    def get_valley_detector(self):
        """
        Function to get the underlying CandidateValleyDetector instance
        """

        return self.valley_detector

"""
polar_histogram_mapper.py

Author: Khushi
"""

import numpy as np
import matplotlib.patches as patches
from matplotlib.collections import PatchCollection

from polar_histogram import PolarHistogram
from candidate_valley_detector import CandidateValleyDetector
from direction_selector import DirectionSelector


class PolarHistogramMapper:
    """
    Mapper class to build and visualize a polar obstacle density histogram
    around the vehicle from LiDAR point cloud data, following the mapping,
    candidate valley detection, and direction selection stages of the
    Vector Field Histogram(VFH) algorithm.

    Each frame, the histogram is drawn as a ring of colored wedges centered
    on the vehicle: denser(more blocked) sectors are drawn longer and closer
    to red, sparser(more open) sectors shorter and closer to green. Sector
    length and color are both normalized by the current frame's maximum
    density, so the ring stays readable regardless of how many obstacles
    are in range. Detected candidate valleys(navigable direction ranges,
    Step 2) are drawn as green arcs just outside that ring. The direction
    selected(Step 3) from those valleys is drawn as a bold blue arrow, with
    a thin dashed line showing the target(goal) direction it was weighed
    against.

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

    def __init__(self, sensor_params=None, num_sectors=72, smoothing_window=5,
                ring_radius_m=8.0, valley_density_threshold=0.2,
                target_x_m=None, target_y_m=None,
                target_weight=1.0, heading_weight=1.0, previous_weight=1.0):
        """
        Constructor
        sensor_params: LiDAR's SensorParameters object, used for max sensing range
        num_sectors: Number of angular sectors dividing 360[deg](resolution)
        smoothing_window: Half-width of the triangular smoothing filter(sectors)
        ring_radius_m: Drawing radius of the densest sector's wedge on the global plot[m]
        valley_density_threshold: Smoothed density at/below which a sector counts as navigable
        target_x_m, target_y_m: Fixed goal point[m] the target direction points toward.
                                 When not given, the vehicle's current heading is used as
                                 the target direction(i.e. "keep going straight")
        target_weight, heading_weight, previous_weight: Direction selection cost weights
        """

        max_range_m = sensor_params.MAX_RANGE_M if sensor_params else 40.0

        self.histogram = PolarHistogram(num_sectors=num_sectors,
                                        max_range_m=max_range_m,
                                        smoothing_window=smoothing_window)
        self.valley_detector = CandidateValleyDetector(density_threshold=valley_density_threshold)
        self.direction_selector = DirectionSelector(target_weight=target_weight,
                                                     heading_weight=heading_weight,
                                                     previous_weight=previous_weight)
        self.ring_radius_m = ring_radius_m
        self.target_x_m = target_x_m
        self.target_y_m = target_y_m

        self.vehicle_x_m = 0.0
        self.vehicle_y_m = 0.0
        self.vehicle_yaw_rad = 0.0
        self.target_angle_rad = 0.0

    def update(self, point_cloud, state):
        """
        Function to update the polar histogram, candidate valleys, and
        selected direction from the latest LiDAR point cloud
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

        self.target_angle_rad = self._target_angle_rad()
        self.direction_selector.select(self.valley_detector.get_valleys(),
                                       self.vehicle_yaw_rad, self.target_angle_rad)

    def _target_angle_rad(self):
        """
        Private function to get the current target direction in the global
        frame[rad]: toward(target_x_m, target_y_m) if given, otherwise
        straight ahead(the vehicle's current heading)
        """

        if self.target_x_m is None or self.target_y_m is None:
            return self.vehicle_yaw_rad

        return np.arctan2(self.target_y_m - self.vehicle_y_m,
                          self.target_x_m - self.vehicle_x_m)

    def draw(self, axes, elems):
        """
        Function to draw the polar histogram ring, candidate valley arcs,
        and the selected/target direction around the vehicle
        axes: Axes object of figure
        elems: List of plot objects
        """

        self._draw_density_ring(axes, elems)
        self._draw_valleys(axes, elems)
        self._draw_selected_direction(axes, elems)

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

    def _draw_selected_direction(self, axes, elems):
        """
        Private function to draw the target direction as a thin dashed
        line, and(when one was selected) the chosen steering direction as
        a bold arrow, both from the vehicle's position
        axes: Axes object of figure
        elems: List of plot objects
        """

        arrow_length_m = self.ring_radius_m * 1.3

        target_x = self.vehicle_x_m + arrow_length_m * np.cos(self.target_angle_rad)
        target_y = self.vehicle_y_m + arrow_length_m * np.sin(self.target_angle_rad)
        target_line, = axes.plot([self.vehicle_x_m, target_x], [self.vehicle_y_m, target_y],
                                 linestyle="--", color="purple", linewidth=1.0, alpha=0.7)
        elems.append(target_line)

        selected_angle_rad = self.direction_selector.get_selected_angle_rad()
        if selected_angle_rad is None:
            return

        end_x = self.vehicle_x_m + arrow_length_m * np.cos(selected_angle_rad)
        end_y = self.vehicle_y_m + arrow_length_m * np.sin(selected_angle_rad)
        arrow = axes.annotate("", xy=(end_x, end_y), xytext=(self.vehicle_x_m, self.vehicle_y_m),
                              arrowprops=dict(arrowstyle="->", color="blue", linewidth=2.5))
        elems.append(arrow)

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

    def get_direction_selector(self):
        """
        Function to get the underlying DirectionSelector instance
        """

        return self.direction_selector

    def get_target_angle_rad(self):
        """
        Function to get the most recently computed target direction,
        global frame[rad]
        """

        return self.target_angle_rad

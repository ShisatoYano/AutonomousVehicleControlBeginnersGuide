"""
l_shape_fitting_detector.py

Author: Shisato Yano

Reference:
    X. Zhang, W. Xu, C. Dong and J. M. Dolan,
    "Efficient L-Shape Fitting for Vehicle Detection Using Laser Scanners",
    2017 IEEE Intelligent Vehicles Symposium (IV), pp. 54-59, 2017.
"""

import sys
import copy
import itertools
import numpy as np
from collections import deque
from pathlib import Path
from math import sin, cos

abs_dir_path = str(Path(__file__).absolute().parent)
sys.path.append(abs_dir_path + "/../../search/kd_tree")

from kd_tree import KdTree
from rectangle import Rectangle

class LShapeFittingDetector:
    """
    Object detection class by L shape fitting algorithm

    A vehicle seen by a 2D laser scanner looks like an "L": the scanner only
    sees the one or two sides of the car that face it. This class turns a
    point cloud into one bounding rectangle per object in three steps:

    1. Segment: group points that are close to each other into small
       clusters (_adaptive_range_segmentation) and merge clusters that share
       points into one cluster per object (_merge_clusters).
    2. Search the direction: for each cluster, try many rectangle directions
       theta and keep the one that scores best on the "variance criterion"
       (_calculate_variance_criterion).
    3. Build the rectangle: take the minimum and maximum of the points along
       the two axes of that best direction (_calculate_rectangle).

    The direction search and the variance criterion follow Zhang et al. (2017),
    see the reference at the top of this file.
    """

    def __init__(self, min_rng_th_m=3.0, rng_th_rate=0.1, change_angle_deg=1.0):
        """
        Constructor
        min_rng_th_m: Minimum range threshold between 2 points to segment[m]
        rng_th_rate: Ratio of range threshold based on distance
        change_angle_deg: Amount of rectangle direction angle changing to search fitting[deg]
        """
        
        self.MIN_RNG_TH_M = min_rng_th_m
        self.RNG_TH_RATE = rng_th_rate
        self.CHANGE_ANGLE_RAD = np.deg2rad(change_angle_deg)
        self.latest_rectangles_list = []
    
    def _adaptive_range_segmentation(self, point_cloud):
        """
        Private function to segment each points based on adaptive range

        Two points belong to the same small cluster when their distance is
        below a threshold that grows with the distance of the point from the
        sensor, because a laser scanner samples far objects more sparsely:

            r_th = MIN_RNG_TH_M + RNG_TH_RATE * d

        where d[m] is the distance from the sensor to the point being checked.

        point_cloud: Input original point cloud
        Return list of small clusters
        """
        
        clusters_list = []

        points_checked = {point: False for point in point_cloud}
        points_queue = deque()
        
        for point in point_cloud:
            if not points_checked[point]:
                points_queue.append(point)

                while len(points_queue) > 0:
                    popped_point = points_queue.popleft()
                    
                    if not points_checked[popped_point]:
                        point_distance_m = popped_point.get_distance_m()
                        seg_rng_th = self.MIN_RNG_TH_M + self.RNG_TH_RATE * point_distance_m
                        
                        tree = KdTree(point_cloud)
                        neighbor_points = tree.search_neighbor_points_within_r(popped_point, r=seg_rng_th)
                        cluster = set(neighbor_points)
                        
                        points_checked[popped_point] = True
                
                clusters_list.append(cluster)
        
        return clusters_list

    def _merge_clusters(self, cluster_list):
        """
        Private function to merge small clusters into same large cluster

        Neighborhoods found around different points overlap. Whenever two
        clusters have at least one point in common they describe the same
        object, so they are replaced by their union. This repeats until no
        two clusters share a point.

        cluster_list: Small clusters list
        Return list of merged large clusters
        """
        
        merged_list = copy.deepcopy(cluster_list)

        while True:
            changed_list = False

            # permutations of 2 clusters combination
            index_perm_list = list(itertools.permutations(range(len(merged_list)), 2))
            for (index_1, index_2) in index_perm_list:
                if merged_list[index_1] & merged_list[index_2]:
                    merged_list[index_1] = (merged_list[index_1] | merged_list.pop(index_2))
                    changed_list = True
                    break

            if not changed_list: break

        return merged_list
    
    def _rotate_points(self, points_array, angle_rad):
        """
        Private function to rotate points

        Expresses every point (x, y) in a coordinate frame whose first axis
        points in the direction theta (the candidate rectangle direction):

            c1 =  cos(theta) * x + sin(theta) * y
            c2 = -sin(theta) * x + cos(theta) * y

        In this frame the candidate rectangle is axis-aligned, so its four
        edges are simply c1 = min(c1), c1 = max(c1), c2 = min(c2) and
        c2 = max(c2).

        points_array: Input original points, shape (2, N) rows [x, y]
        angle_rad: Rotation angle theta[rad]
        Return rotated points array, shape (2, N) rows [c1, c2]
        """
        
        angle_cos = cos(angle_rad)
        angle_sin = sin(angle_rad)

        rotation_matrix = np.array([[angle_cos, angle_sin], 
                                    [-angle_sin, angle_cos]])
        
        return rotation_matrix @ points_array

    def _calculate_variance_criterion(self, points_array):
        """
        Private function to calculate variance of distance between point and rectangle's edge

        This is the "variance criterion" of Zhang et al. (2017). For a good
        direction, the points of an L-shaped cluster lie along the rectangle's
        edges, so every point is very close to one of them.

        For each point i, with the rectangle spanning [c1_min, c1_max] x
        [c2_min, c2_max], the distance to the nearest edge along each axis is

            d1_i = min(c1_max - c1_i, c1_i - c1_min)
            d2_i = min(c2_max - c2_i, c2_i - c2_min)

        Each point is assigned to the axis whose edge it is closest to:

            E1 = { d1_i : d1_i <  d2_i }      (points on an edge of axis 1)
            E2 = { d2_i : d1_i >= d2_i }      (points on an edge of axis 2)

        and the criterion is the negative total variance of those distances:

            gamma = -Var(E1) - Var(E2)

        If the points hug the edges, the distances are all about the same
        (near zero), the variances are small, and gamma is close to 0, its
        maximum. A poor direction spreads the distances out and makes gamma
        more negative. So the best direction is the one with the LARGEST gamma.

        points_array: Input rotated points, shape (2, N) rows [c1, c2]
        Return gamma, the negative variance of distances (larger is better)
        """
        
        c1 = points_array[0, :]
        c2 = points_array[1, :]

        c1_min, c1_max = min(c1), max(c1)
        c2_min, c2_max = min(c2), max(c2)
        
        d1 = np.minimum(c1_max - c1, c1 - c1_min)
        d2 = np.minimum(c2_max - c2, c2 - c2_min)
        
        e1, e2 = d1[d1 < d2], d2[d1 >= d2]
        v1 = np.var(e1) if len(e1) > 0 else 0
        v2 = np.var(e2) if len(e2) > 0 else 0
        
        gamma = (-v1) + (-v2)
        return gamma

    def _calculate_rectangle(self, points_array):
        """
        Private function to calculate the most fitted rectangle

        Tries a rectangle direction theta every CHANGE_ANGLE_RAD, rotates the
        cluster by theta, scores it with the variance criterion and keeps the
        theta with the largest score. A rectangle looks the same after a
        quarter turn, so only directions in [0, 90 deg) need to be searched.

        With the best theta, the rectangle is the box around the rotated
        points. Its four edges are lines a*x + b*y = c in the original frame:

            cos(theta)*x + sin(theta)*y = c1_min      cos(theta)*x + sin(theta)*y = c1_max
           -sin(theta)*x + cos(theta)*y = c2_min     -sin(theta)*x + cos(theta)*y = c2_max

        which are the coefficient lists a, b, c given to Rectangle below.

        points_array: Input points in a cluster
        Return calculated rectangle object
        """
        
        # (best criterion so far, its angle); the criterion is maximized, so it starts at -inf
        min_cost_angle = (-float("inf"), None)
        initial_angle_rad = 0.0
        end_angle_rad = np.pi / 2.0 - self.CHANGE_ANGLE_RAD
        
        # search the most fitted angle of rectangle's direction
        for angle_rad in np.arange(initial_angle_rad, end_angle_rad, self.CHANGE_ANGLE_RAD):
            rotated_points = self._rotate_points(points_array, angle_rad)
            cost = self._calculate_variance_criterion(rotated_points)
            if min_cost_angle[0] < cost: min_cost_angle = (cost, angle_rad)
        most_fitting_points = self._rotate_points(points_array, min_cost_angle[1])
        
        # create fitting rectangle object
        angle_cos, angle_sin = cos(min_cost_angle[1]), sin(min_cost_angle[1])
        c1, c2 = most_fitting_points[0, :], most_fitting_points[1, :]
        rectangle = Rectangle(a=[angle_cos, -angle_sin, angle_cos, -angle_sin],
                              b=[angle_sin, angle_cos, angle_sin, angle_cos],
                              c=[min(c1), min(c2), max(c1), max(c2)])
        
        return rectangle

    def _search_rectangles(self, clusters_list):
        """
        Private function to search the fitted rectangle of each clusters
        cluster_list: List of large clusters
        """
        
        rectangles_list = []

        for cluster in clusters_list:
            array_list = [point.get_point_array() for point in list(cluster)]
            integrated_array = np.concatenate(array_list, 1)
            rectangles_list.append(self._calculate_rectangle(integrated_array))

        self.latest_rectangles_list = rectangles_list

    def update(self, point_cloud):
        """
        Function to update each rectangle data
        point_cloud: Input original point cloud
        """
        
        clusters_list = self._adaptive_range_segmentation(point_cloud)

        merged_clusters_list = self._merge_clusters(clusters_list)

        self._search_rectangles(merged_clusters_list)
    
    def draw(self, axes, elems, x_m, y_m, yaw_rad):
        """
        Function to draw rectangle of each clusters
        axes: Axes object of figure
        elems: List of plot objects
        x_m: Vehicle's position x[m]
        y_m: Vehicle's position y[m]
        angle_rad: Vehicle's yaw angle[rad]
        """
        
        for rectangle in self.latest_rectangles_list:
            rectangle.draw(axes, elems, x_m, y_m, yaw_rad)

"""
l_shape_fitting_detector.py

Author: Shisato Yano
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
    """

    def __init__(self, min_rng_th_m=3.0, rng_th_rate=0.1, change_angle_deg=1.0):
        self.MIN_RNG_TH_M = min_rng_th_m
        self.RNG_TH_RATE = rng_th_rate
        self.CHANGE_ANGLE_RAD = np.deg2rad(change_angle_deg)
        self.latest_rectangles_list = []
    
    def _adaptive_range_segmentation(self, point_cloud):
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

    def _mearge_clusters(self, cluster_list):
        mearged_list = copy.deepcopy(cluster_list)

        while True:
            changed_list = False

            index_perm_list = list(itertools.permutations(range(len(mearged_list)), 2))
            for (index_1, index_2) in index_perm_list:
                if mearged_list[index_1] & mearged_list[index_2]:
                    mearged_list[index_1] = (mearged_list[index_1] | mearged_list.pop(index_2))
                    changed_list = True
                    break

            if not changed_list: break

        return mearged_list
    
    def _rotate_points(self, points_array, angle_rad):
        angle_cos = cos(angle_rad)
        angle_sin = sin(angle_rad)

        rotation_matrix = np.array([[angle_cos, angle_sin], 
                                    [-angle_sin, angle_cos]])
        
        return rotation_matrix @ points_array

    def _calculate_variance_criterion(self, points_array):
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
        min_cost_angle = (-float("inf"), None)
        initial_angle_rad = 0.0
        end_angle_rad = np.pi / 2.0 - self.CHANGE_ANGLE_RAD
        
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
        rectangles_list = []

        for cluster in clusters_list:
            array_list = [point.get_point_array() for point in list(cluster)]
            integrated_array = np.concatenate(array_list, 1)
            rectangles_list.append(self._calculate_rectangle(integrated_array))

        self.latest_rectangles_list = rectangles_list

    def update(self, point_cloud):
        clusters_list = self._adaptive_range_segmentation(point_cloud)

        mearged_clusters_list = self._mearge_clusters(clusters_list)

        self._search_rectangles(mearged_clusters_list)

"""
l_shape_fitting_detector.py

Author: Shisato Yano
"""

import sys
import copy
import itertools
from collections import deque
from pathlib import Path

abs_dir_path = str(Path(__file__).absolute().parent)
sys.path.append(abs_dir_path + "/../../search/kd_tree")

from kd_tree import KdTree

class LShapeFittingDetector:
    """
    Object detection class by L shape fitting algorithm
    """

    def __init__(self, min_rng_th_m=3.0, rng_th_rate=0.1):
        self.MIN_RNG_TH_M = min_rng_th_m
        self.RNG_TH_RATE = rng_th_rate
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

    def _search_rectangles(self, clusters_list):
        rectangles_list = []

        for cluster in clusters_list:
            print(len(cluster))

        self.latest_rectangles_list = rectangles_list

    def update(self, point_cloud):
        clusters_list = self._adaptive_range_segmentation(point_cloud)

        mearged_clusters_list = self._mearge_clusters(clusters_list)

        self._search_rectangles(mearged_clusters_list)

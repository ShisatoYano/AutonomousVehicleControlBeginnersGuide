"""
l_shape_fitting_detector.py

Author: Shisato Yano
"""

from collections import deque

class LShapeFittingDetector:
    """
    Object detection class by L shape fitting algorithm
    """

    def __init__(self, min_rng_th_m=0.5, rng_th_rate=0.1):
        self.MIN_RNG_TH_M = min_rng_th_m
        self.RNG_TH_RATE = rng_th_rate
    
    def _adaptive_range_segmentation(self, point_cloud):
        clusters_list = []

        points_checked = {point: False for point in point_cloud}
        points_queue = deque()
        
        for point in point_cloud:
            if not points_checked[point]:
                cluster = set()
                points_queue.append(point)

                while len(points_queue) > 0:
                    popped_point = points_queue.popleft()
                    if not points_checked[popped_point]:
                        point_distance_m = popped_point.get_distance_m()
                        seg_rng_th = self.MIN_RNG_TH_M + self.RNG_TH_RATE * point_distance_m
                        print(point_distance_m, seg_rng_th)
                

    def update(self, point_cloud):
        self._adaptive_range_segmentation(point_cloud)

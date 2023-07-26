"""
kd_tree.py

Ahthor: Shisato Yano
"""

# import path setting
import numpy as np
import sys
from pathlib import Path
import matplotlib.pyplot as plt
import matplotlib.patches as ptc
import time

abs_dir_path = str(Path(__file__).absolute().parent)
sys.path.append(abs_dir_path + "/../../sensors/lidar")


# import component modules
from scan_point import ScanPoint


class Node:
    """
    Tree's node class
    """
    
    def __init__(self):
        """
        Constructor
        """
        
        self.axis = 0
        self.data = None
        self.left_child = None
        self.right_child = None


class KdTree:
    """
    Searching by kd-tree algorithm class
    """

    def __init__(self, scan_points_list):
        """
        Constructor
        scan_points_list: list of ScanPoint object from LiDAR/Radar
        """

        self.root = self._build_tree(scan_points_list)
        self.candidates_stack = []
        
    
    def _build_tree(self, scan_points_list, depth=0):
        """
        Private function to build kd-tree
        scan_points_list: list of scan points from LiDAR/Radar
        depth: depth of kd-tree
        """
        
        if not scan_points_list: return None

        k = scan_points_list[0].get_dimension()
        axis = depth % k

        scan_points_list.sort(key=lambda x:x.get_point_array()[axis])
        median = len(scan_points_list) // 2

        node = Node()
        node.axis = axis
        node.data = scan_points_list[median]
        node.left_child = self._build_tree(scan_points_list[0:median], depth+1)
        node.right_child = self._build_tree(scan_points_list[median+1:], depth+1)

        return node

    def _search_edge_node(self, target_point_array, node):
        if not node.left_child and not node.right_child: return node

        axis = node.axis
        if target_point_array[axis] <= node.data.get_point_array()[axis]:
            if node.left_child:
                return self._search_edge_node(target_point_array, node.left_child)
            else:
                return node
        else:
            if node.right_child:
                return self._search_edge_node(target_point_array, node.right_child)
            else:
                return node
    
    def _search_nearest_neighbor_candidates(self, target_point_array, search_radius, node):
        if node:
            axis = node.axis
            diff_axis = abs(target_point_array[axis] - node.data.get_point_array()[axis])
            if search_radius > diff_axis: 
                self.candidates_stack.append(node)
            self._search_nearest_neighbor_candidates(target_point_array, search_radius, node.left_child)
            self._search_nearest_neighbor_candidates(target_point_array, search_radius, node.right_child)

    def search_nearest_neighbor_point(self, target_point):
        """
        Find the nearest neighbor point against target point by kd-tree
        """
        
        target_point_array = target_point.get_point_array()

        edge_node = self._search_edge_node(target_point_array, self.root)
        search_radius = np.linalg.norm(target_point_array - edge_node.data.get_point_array())

        self._search_nearest_neighbor_candidates(target_point_array, search_radius, self.root)
        min_dist = search_radius
        nearest_neighbor_node = edge_node

        while self.candidates_stack:
            candidate_node = self.candidates_stack.pop()
            candidate_dist = np.linalg.norm(target_point_array - candidate_node.data.get_point_array())
            if min_dist > candidate_dist:
                min_dist = candidate_dist
                nearest_neighbor_node = candidate_node

        return nearest_neighbor_node.data

    def search_neighbor_points_within_r(self, target_point, r=0.2):
        target_point_array = target_point.get_point_array()

        self._search_nearest_neighbor_candidates(target_point_array, r, self.root)

        neighbor_points = []
        while self.candidates_stack:
            candidate_node = self.candidates_stack.pop()
            candidate_dist = np.linalg.norm(target_point_array - candidate_node.data.get_point_array())
            if r > candidate_dist:
                neighbor_points.append(candidate_node.data)

        return neighbor_points


def main():
    np.random.seed(20)
    
    points_xy = np.random.random((100, 2))
    point_cloud = [ScanPoint(0.0, 0.0, xy[0], xy[1]) for xy in points_xy]

    targets_xy = np.random.random((1, 2))
    target_points = [ScanPoint(0.0, 0.0, xy[0], xy[1]) for xy in targets_xy]

    for point in point_cloud:
        point_xy = point.get_point_array()
        plt.plot(point_xy[0], point_xy[1], "og")

    kd_tree = KdTree(point_cloud)

    for target_point in target_points:
        # nearest_point = kd_tree.search_nearest_neighbor_point(target_point)
        # nearest_xy = nearest_point.get_point_array()

        neighbor_points = kd_tree.search_neighbor_points_within_r(target_point)
        
        target_xy = target_point.get_point_array()
        plt.plot(target_xy[0], target_xy[1], "xb")  
        # plt.plot([nearest_xy[0], target_xy[0]], [nearest_xy[1], target_xy[1]], "-r")

        for neighbor_point in neighbor_points:
            neighbor_xy = neighbor_point.get_point_array()
            plt.plot([neighbor_xy[0], target_xy[0]], [neighbor_xy[1], target_xy[1]], "-r")
    
    plt.axis("square")
    plt.show()


if __name__ == "__main__":
    main()

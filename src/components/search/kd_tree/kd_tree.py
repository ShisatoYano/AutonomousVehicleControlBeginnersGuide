"""
kd_tree.py

Ahthor: Shisato Yano
"""

# import path setting
import numpy as np
import sys
from pathlib import Path
import matplotlib.pyplot as plt
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
        self.stack = []
        
    
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
    
    def _push_stack(self, target_point_array, node):
        """
        Private function to push node into stack.
        Pushed nodes are candidates of nearest neighbor point with target
        """
        
        if node:
            self.stack.append(node)
            axis = node.axis
            if target_point_array[axis] < node.data.get_point_array()[axis]:
                self._push_stack(target_point_array, node.left_child)
            else:
                self._push_stack(target_point_array, node.right_child)

    def search_nearest_neighbor_point(self, target_point):
        target_point_array = target_point.get_point_array()

        self._push_stack(target_point_array, self.root)

        nearest_node = None

        while self.stack:
            node = self.stack.pop()
            axis = node.axis
            dist = np.linalg.norm(target_point_array - node.data.get_point_array())

            if not nearest_node:
                nearest_node = node
                min_dist = dist
            else:
                diff_axis = abs(target_point_array[axis] - node.data.get_point_array()[axis])
                if diff_axis <= min_dist:
                    if min_dist > dist:
                        min_dist = dist
                        nearest_node = node

        return nearest_node.data


def main():
    np.random.seed(20)
    
    points_xy = np.random.random((100, 2))
    point_cloud = [ScanPoint(0.0, 0.0, xy[0], xy[1]) for xy in points_xy]

    targets_xy = np.random.random((5, 2))
    target_points = [ScanPoint(0.0, 0.0, xy[0], xy[1]) for xy in targets_xy]

    for point in point_cloud:
        point_xy = point.get_point_array()
        plt.plot(point_xy[0], point_xy[1], "og")

    # target_point_1 = ScanPoint(0.0, 0.0, 0.035, 0.51)
    # target_point_2 = ScanPoint(0.0, 0.0, 0.05, 0.9)
    # target_point_3 = ScanPoint(0.0, 0.0, 0.6, 0.5)
    # target_point_4 = ScanPoint(0.0, 0.0, 0.8, 0.9)

    kd_tree = KdTree(point_cloud)

    for target_point in target_points:
        sta_time = time.perf_counter()
        nearest_point = kd_tree.search_nearest_neighbor_point(target_point)
        end_time = time.perf_counter()
        print(end_time - sta_time)
        nearest_xy = nearest_point.get_point_array()
        
        target_xy = target_point.get_point_array()
        plt.plot(target_xy[0], target_xy[1], "xb")

        plt.plot([nearest_xy[0], target_xy[0]], [nearest_xy[1], target_xy[1]], "-r")
    
    plt.show()


if __name__ == "__main__":
    main()

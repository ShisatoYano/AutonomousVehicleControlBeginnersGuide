"""
kd_tree.py

Ahthor: Shisato Yano
"""

# import path setting
import numpy as np
import sys
from pathlib import Path

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
        self.location = None
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
        node.location = scan_points_list[median]
        node.left_child = self._build_tree(scan_points_list[0:median], depth+1)
        node.right_child = self._build_tree(scan_points_list[median+1:], depth+1)

        return node


def main():
    np.random.seed(20)
    
    points_xy = np.random.random((10, 2))
    point_cloud = [ScanPoint(0.0, 0.0, xy[0], xy[1]) for xy in points_xy]

    kd_tree = KdTree(point_cloud)


if __name__ == "__main__":
    main()

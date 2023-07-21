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
    
    def search_nearest_neighbor_point(self, target_point):
        target = target_point.get_point_array()

        nodes_stack = [self.root]
        while nodes_stack:
            node = nodes_stack.pop()
            axis = node.axis
            
            print(node.data.get_point_array()[0], node.data.get_point_array()[1])
            
            if target[axis] < node.data.get_point_array()[axis]:
                if node.left_child: nodes_stack.append(node.left_child)
            else:
                if node.right_child: nodes_stack.append(node.right_child)

            # if node.left_child:
            #     nodes_stack.append(node.left_child)
            # if node.right_child:
            #     nodes_stack.append(node.right_child)


def main():
    np.random.seed(20)
    
    points_xy = np.random.random((10, 2))
    point_cloud = [ScanPoint(0.0, 0.0, xy[0], xy[1]) for xy in points_xy]

    target_point_1 = ScanPoint(0.0, 0.0, 0.035, 0.51)
    target_point_2 = ScanPoint(0.0, 0.0, 0.65, 0.70)
    target_point_3 = ScanPoint(0.0, 0.0, 0.65, 0.03)

    kd_tree = KdTree(point_cloud)

    kd_tree.search_nearest_neighbor_point(target_point_1)


if __name__ == "__main__":
    main()

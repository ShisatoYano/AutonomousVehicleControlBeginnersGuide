"""
search_neighbor_points_kd_tree.py

Author: Shisato Yano
"""

# import path setting
import numpy as np
import sys
import matplotlib.pyplot as plt
from pathlib import Path

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"

sys.path.append(abs_dir_path + relative_path + "search/kd_tree")
sys.path.append(abs_dir_path + relative_path + "sensors/lidar")

# import component modules
from kd_tree import KdTree
from scan_point import ScanPoint


# flag to show plot figure
# when executed as unit test, this flag is set as false
show_plot = True


def main():
    """
    Main process function
    """

    fig = plt.figure()
    ax = plt.axes()

    # generate point cloud data
    np.random.seed(10)

    # source point cloud
    source_xy_list = np.random.random((100, 2))
    source_point_cloud = [ScanPoint(0, 0, xy[0], xy[1]) for xy in source_xy_list]

    for source_point in source_point_cloud:
        source_xy = source_point.get_point_array()
        ax.plot(source_xy[0], source_xy[1], "ob")

    # target point cloud
    target_xy_list = np.random.random((3, 2))
    target_point_cloud = [ScanPoint(0, 0, xy[0], xy[1]) for xy in target_xy_list]

    kd_tree = KdTree(source_point_cloud)

    for target_point in target_point_cloud:
        target_xy = target_point.get_point_array()
        ax.plot(target_xy[0], target_xy[1], "xr")

        neighbor_points = kd_tree.search_neighbor_points_within_r(target_point)
        for neighbor_point in neighbor_points:
            neighbor_xy = neighbor_point.get_point_array()
            ax.plot([neighbor_xy[0], target_xy[0]], [neighbor_xy[1], target_xy[1]], "-g")

    ax.set_aspect("equal")
    ax.grid()
    ax.set_title("Blue o: Source points, Red x: Target points, Green -: Neighbor line")

    if show_plot: plt.show()


if __name__ == "__main__":
    main()

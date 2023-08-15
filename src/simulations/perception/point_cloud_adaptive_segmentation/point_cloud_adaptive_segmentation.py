"""
point_cloud_adaptive_segmentation.py

Author: Shisato Yano
"""

# import path setting
import numpy as np
import sys
import matplotlib.pyplot as plt
from pathlib import Path

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"

sys.path.append(abs_dir_path + relative_path + "sensors/lidar")

# import component modules
from scan_point import ScanPoint


# flag to show plot figure
# when executed as unit test, this flag is set as false
show_plot = True

# number of points per cluster
POINTS_NUM_CLUSTER = 10


def main():
    """
    Main process function
    """

    fig = plt.figure()
    ax = plt.axes()

    # generate point cloud data
    np.random.seed(10)
    point_cloud_list = []

    # center points
    center_xy_list = np.random.random((3, 2))
    for xy in center_xy_list:
        print(xy[0], xy[1])
    # target_point_cloud = [ScanPoint(0, 0, xy[0], xy[1]) for xy in center_xy_list]

    # for target_point in target_point_cloud:
    #     target_xy = target_point.get_point_array()
    #     ax.plot(target_xy[0], target_xy[1], "xr")

    ax.set_aspect("equal")
    ax.grid()
    ax.set_title("Blue o: Source points, Red x: Target points, Green -: Neighbor line")

    if show_plot: plt.show()


if __name__ == "__main__":
    main()

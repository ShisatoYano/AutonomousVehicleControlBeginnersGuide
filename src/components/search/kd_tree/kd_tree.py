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


def main():
    np.random.seed(20)
    
    points_xy = np.random.random((100, 2))
    point_cloud = [ScanPoint(0.0, 0.0, xy[0], xy[1]) for xy in points_xy]


if __name__ == "__main__":
    main()

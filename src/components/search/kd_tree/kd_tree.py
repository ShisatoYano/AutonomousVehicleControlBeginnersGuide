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
    print("kd_tree")


if __name__ == "__main__":
    main()

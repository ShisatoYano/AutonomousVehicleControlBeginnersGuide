"""
Test of sensing simulation by Omni directional LiDAR

Author: Shisato Yano
"""

from pathlib import Path
import sys

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/perception/lidar_obstacle_sensing")
import lidar_obstacle_sensing


def test_simulation():
    lidar_obstacle_sensing.show_plot = False

    lidar_obstacle_sensing.main()
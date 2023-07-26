"""
Unit test of searching nearest neighbor point simulation by k-d tree

Author: Shisato Yano
"""

import matplotlib as mpl
mpl.use("Agg")
from pathlib import Path
import sys

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/perception/point_cloud_search")
import search_nearest_neighbor_kd_tree


def test_simulation():
    search_nearest_neighbor_kd_tree.show_plot = False

    search_nearest_neighbor_kd_tree.main()

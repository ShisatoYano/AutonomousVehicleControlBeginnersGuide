"""
ndt_map.py

Author: Shisato Yano
"""

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../grid")
from grid_map import GridMap
from ndt_grid import NdtGrid


class NdtMap:
    """
    NDT grid map class
    """

    def __init__(self, width_m=60.0, height_m=60.0, resolution_m=5.0,
                 center_x_m=0.0, center_y_m=0.0):
        """
        Constructor
        """

        self.map = GridMap(width_m, height_m, resolution_m, 
                           center_x_m, center_y_m, init_grid=NdtGrid())

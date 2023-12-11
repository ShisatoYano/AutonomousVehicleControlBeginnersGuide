"""
ndt_mapper.py

Author: Shisato Yano
"""

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../grid")
from grid_mapper import GridMapper


class NdtMapper:
    """
    NDT grid map construnction class
    """

    def __init__(self, width_m=60.0, height_m=60.0, resolution_m=5.0,
                 center_x_m=0.0, center_y_m=0.0):
        """
        Constructor
        """

        self.map = GridMapper(width_m, height_m, resolution_m, 
                              center_x_m, center_y_m)

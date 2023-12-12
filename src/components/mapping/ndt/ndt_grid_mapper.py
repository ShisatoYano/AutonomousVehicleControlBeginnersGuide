"""
ndt_grid_mapper.py

Author: Shisato Yano
"""

from ndt_map import NdtMap


class NdtGridMapper:
    """
    NDT grid map construction class
    """

    def __init__(self, width_m=60.0, height_m=60.0, resolution_m=5.0,
                 center_x_m=0.0, center_y_m=0.0):
        """
        Constructor
        """

        self.map = NdtMap(width_m, height_m, resolution_m, center_x_m, center_y_m)

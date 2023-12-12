"""
ndt_global_mapper.py

Author: Shisato Yano
"""

from ndt_map import NdtMap


class NdtGlobalMapper:
    """
    NDT global map construction class
    """

    def __init__(self, width_m=60.0, height_m=60.0, resolution_m=5.0,
                 center_x_m=0.0, center_y_m=0.0):
        """
        Constructor
        width_m: Width size of map[m]
        height_m: Height size of map[m]
        resolution: Size of each grids[m]
        center_x_m: Center x position of map[m]
        center_y_m: Center y position of map[m]
        """

        self.map = NdtMap(width_m, height_m, resolution_m, center_x_m, center_y_m)
    
    def update(self, point_cloud, state):
        print(len(point_cloud))

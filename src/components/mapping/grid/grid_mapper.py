"""
grid_mapper.py

Author: Shisato Yano
"""

from float_grid import FloatGrid


class GridMapper:
    """
    Grid map construnction class
    """

    def __init__(self, width_m=60.0, height_m=60.0, resolution_m=5.0,
                 center_x_m=0.0, center_y_m=0.0, init_grid=FloatGrid(0.0)):
        """
        Constructor
        width_m: Width size of map[m]
        height_m: Height size of map[m]
        resolution: Size of each grids[m]
        center_x_m: Center x position of map[m]
        center_y_m: Center y position of map[m]
        """

        self.width_m = width_m
        self.height_m = height_m
        self.resolution_m = resolution_m
        self.center_x_m = center_x_m
        self.center_y_m = center_y_m

        self.width_grids_num = int(round(self.width_m / self.resolution_m))
        self.height_grids_num = int(round(self.height_m / self.resolution_m))
        self.all_grids_num = self.width_grids_num * self.height_grids_num

        self.left_bottom_x_m = self.center_x_m - self.width_m / 2.0
        self.left_bottom_y_m = self.center_y_m - self.height_m / 2.0

        self.map = [init_grid] * self.all_grids_num

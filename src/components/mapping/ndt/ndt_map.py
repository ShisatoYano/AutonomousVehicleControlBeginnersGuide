"""
ndt_map.py

Author: Shisato Yano
"""

import sys
from pathlib import Path
from collections import defaultdict

sys.path.append(str(Path(__file__).absolute().parent) + "/../grid")
from grid_map import GridMap
from ndt_grid import NdtGrid


class NdtMap:
    """
    NDT grid map class
    """

    def __init__(self, width_m=60.0, height_m=60.0, resolution_m=5.0,
                 center_x_m=0.0, center_y_m=0.0, min_points_num=3):
        """
        Constructor
        width_m: Width size of map[m]
        height_m: Height size of map[m]
        resolution: Size of each grids[m]
        center_x_m: Center x position of map[m]
        center_y_m: Center y position of map[m]
        min_points_num: Minimum number of points for normal distribution transform
        """

        self.map = GridMap(width_m, height_m, resolution_m, 
                           center_x_m, center_y_m, init_grid=NdtGrid())
        
        self.MIN_POINTS_NUM = min_points_num
    
    def _create_grid_points_index_map(self, points_xy_list):
        """
        Function to create map between grid index and points index
        points_xy_list: List of point cloud x-y coordinates
        Return: Map between grid index and points index
        """
        
        grid_index_map = defaultdict(list)

        for i in range(len(points_xy_list)):
            point_xy = points_xy_list[i]
            index = self.map.calculate_vector_index_from_position(point_xy[0, 0], point_xy[1, 0])
            grid_index_map[index].append(i)
        
        return grid_index_map

    def update_map(self, points_xy_list):
        """
        Function to update map
        points_xy_list: List of point cloud x-y coordinates
        """
        
        grid_index_map = self._create_grid_points_index_map(points_xy_list)

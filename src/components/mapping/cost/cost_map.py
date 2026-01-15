"""
cost_map.py

Author: Bhavesh Lokesh Agarwal
"""

import numpy as np
import sys
from pathlib import Path
import matplotlib.patches as patches
import matplotlib.cm as cm
sys.path.append(str(Path(__file__).absolute().parent) + "/../grid")
from grid_map import GridMap
from grid_map import FloatGrid


class CostMap:
    """
    Cost grid map class
    """

    def __init__(self, width_m=60.0, height_m=60.0, resolution_m=1.0,
                 center_x_m=0.0, center_y_m=0.0, max_cost=100.0, 
                 obstacle_cost=50.0, free_cost=0.0):
        """
        Constructor
        width_m: Width size of map[m]
        height_m: Height size of map[m]
        resolution_m: Size of each cells[m]
        center_x_m: Center x position of map[m]
        center_y_m: Center y position of map[m]
        max_cost: Maximum cost value
        obstacle_cost: Cost value for obstacles
        free_cost: Cost value for free space
        """
        
        self.map = GridMap(width_m, height_m, resolution_m,
                           center_x_m, center_y_m)
        self.max_cost = max_cost
        self.obstacle_cost = obstacle_cost
        self.free_cost = free_cost

    def update_map(self, points_x_list, points_y_list, cost_increment=1.0):
        """
        Function to update cost map by adding cost to cells with obstacles
        points_x_list: List of x coordinates of point cloud
        points_y_list: List of y coordinates of point cloud
        cost_increment: Cost value to add per hit
        """

        points_x_array, points_y_array = np.array(points_x_list), np.array(points_y_list)
        
        cells_hit = {}
        
        for i in range(len(points_x_array)):
            index = self.map.calculate_vector_index_from_position(points_x_array[i],
                                                                  points_y_array[i])
            if index is not None:
                if index not in cells_hit:
                    cells_hit[index] = 0
                cells_hit[index] += 1
        
        for index, hit_count in cells_hit.items():
            current_grid = self.map.get_grid_data(index)
            current_cost = current_grid.get_data()
            
            # Increment cost based on hits, but cap at max_cost
            new_cost = min(current_cost + (cost_increment * hit_count), self.max_cost)
            self.map.set_grid_data(index, FloatGrid(value=new_cost))

    def set_cost(self, x_m, y_m, cost_value):
        """
        Function to set cost value at a specific position
        x_m: x coordinate position[m]
        y_m: y coordinate position[m]
        cost_value: Cost value to set
        """
        
        index = self.map.calculate_vector_index_from_position(x_m, y_m)
        if index is not None:
            clamped_cost = max(self.free_cost, min(cost_value, self.max_cost))
            self.map.set_grid_data(index, FloatGrid(value=clamped_cost))

    def get_cost(self, x_m, y_m):
        """
        Function to get cost value at a specific position
        x_m: x coordinate position[m]
        y_m: y coordinate position[m]
        Return: Cost value at position, or None if out of bounds
        """
        
        index = self.map.calculate_vector_index_from_position(x_m, y_m)
        if index is not None:
            return self.map.get_grid_data(index).get_data()
        return None

    def draw_map(self, axes, elems, colormap='RdYlGn_r'):
        """
        Function to draw cost map data with color gradient
        axes: Axes object of figure
        elems: List of plot object
        colormap: Matplotlib colormap name (default: 'RdYlGn_r' for red-yellow-green reversed)
        """

        # Normalize cost values for colormap (0 to 1)
        cost_normalizer = cm.ScalarMappable(cmap=colormap)
        cost_normalizer.set_clim(vmin=self.free_cost, vmax=self.max_cost)
        
        for vector_idx in range(self.map.all_grids_num):
            grid_data = self.map.get_grid_data(vector_idx)
            cost_value = grid_data.get_data()
            
            if cost_value > self.free_cost:
                center_x, center_y = self.map.calculate_grid_center_xy_pos_from_vector_index(vector_idx)
                bottom_left_x = center_x - self.map.resolution_m / 2.0
                bottom_left_y = center_y - self.map.resolution_m / 2.0
                
                # Get color based on cost value
                rgba = cost_normalizer.to_rgba(cost_value)
                
                rect = patches.Rectangle((bottom_left_x, bottom_left_y),
                                        self.map.resolution_m,
                                        self.map.resolution_m,
                                        linewidth=0.1,
                                        edgecolor='k',
                                        facecolor=rgba,
                                        alpha=0.7)
                axes.add_patch(rect)
                elems.append(rect)


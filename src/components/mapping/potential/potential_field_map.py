"""
potential_field_map.py

Author: Panav Arpit Raaj
"""

import numpy as np
import sys
from pathlib import Path
import matplotlib.patches as patches
import matplotlib.cm as cm
sys.path.append(str(Path(__file__).absolute().parent) + "/../grid")
from grid_map import GridMap
from grid_map import FloatGrid


class PotentialFieldMap:
    """
    Potential field map class
    """                                                           

    def __init__(self, width_m=60.0, height_m=60.0, resolution_m=1.0,
                 center_x_m=0.0, center_y_m=0.0, zeta=0.001, eta=10.0, rho=4.0,
                 goal_x_m=0.0, goal_y_m=0.0):
        """
        Constructor
        zeta: Attractive scaling gain
        eta: Repulsive scaling gain
        rho: Obstacle Influence distance
        goal_x_m: Goal x position of map[m]
        goal_y_m: Goal y position of map[m]
        """
        
        self.map = GridMap(width_m, height_m, resolution_m,
                           center_x_m, center_y_m)
        self.zeta = zeta
        self.eta = eta
        self.rho = rho
        self.goal_x_m = goal_x_m
        self.goal_y_m = goal_y_m
        self.max_cost = 1000.0
        self.free_cost = 0.0


    def update_map(self, points_x_list, points_y_list):
        """
        Function to update potential field map
        points_x_list: List of x coordinates of point cloud
        points_y_list: List of y coordinates of point cloud
        """

        obstacle_positions = list(zip(points_x_list, points_y_list))
        
        for vector_idx in range(self.map.all_grids_num):
            center_x, center_y = self.map.calculate_grid_center_xy_pos_from_vector_index(vector_idx)
            
            u_att = self.calculate_attractive_potential(center_x, center_y)
            u_rep = self.calculate_repulsive_potential(center_x, center_y, obstacle_positions)
            total_potential = min(u_att + u_rep, self.max_cost)
            self.map.set_grid_data(vector_idx, FloatGrid(value=total_potential))




    def calculate_attractive_potential(self, x_m, y_m):
        """
        Function to calculate attractive potential at a specific position
        x_m: x coordinate position[m]
        y_m: y coordinate position[m]
        Return: Attractive potential at position
        """
        
        d_squared = (x_m - self.goal_x_m)**2 + (y_m - self.goal_y_m)**2
        return 0.5 * self.zeta * d_squared

    def calculate_repulsive_potential(self, x_m, y_m, obstacle_positions):
        """
        Function to calculate total repulsive potential at a specific position
        x_m: x coordinate position[m]
        y_m: y coordinate position[m]
        obstacle_positions: List of (x, y) tuples for obstacle positions
        Return: Total repulsive potential at position
        """
        u_rep_total = 0.0
        eps = 1e-9
        for obs_x_m, obs_y_m in obstacle_positions:
            d = np.sqrt((x_m - obs_x_m)**2 + (y_m - obs_y_m)**2)
            if d <= self.rho:
                u_rep_total += 0.5 * self.eta * (1.0/(d + eps) - 1.0/self.rho)**2
        return u_rep_total

    def draw_map(self, axes, elems, colormap='jet'):
        """
        Function to draw cost map data with color gradient using pcolormesh
        axes: Axes object of figure
        elems: List of plot object
        colormap: Matplotlib colormap name
        """
        
        # Create grid coordinates for pcolormesh
        x_range = np.arange(self.map.width_grids_num + 1) * self.map.resolution_m + self.map.left_bottom_x_m
        y_range = np.arange(self.map.height_grids_num + 1) * self.map.resolution_m + self.map.left_bottom_y_m
        X, Y = np.meshgrid(x_range, y_range)
        
        # Reshape grid data into 2D array (height x width)
        Z = np.zeros((self.map.height_grids_num, self.map.width_grids_num))
        
        for vector_idx in range(self.map.all_grids_num):
            val = self.map.get_grid_data(vector_idx).get_data()
            if val > self.free_cost:
                x_idx, y_idx = self.map.calculate_xy_index_from_vector_index(vector_idx)
                Z[y_idx][x_idx] = val
        
        # Use pcolormesh for efficient heatmap rendering
        # vmin/vmax will auto-scale the colors
        pcm = axes.pcolormesh(X, Y, Z, cmap=colormap, alpha=0.5, shading='flat')
        elems.append(pcm)


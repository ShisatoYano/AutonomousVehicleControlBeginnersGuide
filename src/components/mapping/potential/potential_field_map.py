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
    Potential field map class with persistent obstacle memory.

    The field maintains a memory of all previously observed obstacles,
    ensuring stable path planning even when obstacles are outside the
    current sensor field of view.
    
    Uses incremental updates for efficiency - only recalculates the
    repulsive field for cells affected by newly observed obstacles.
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

        # Persistent obstacle memory: stores grid indices of observed obstacles
        # Using a set for O(1) lookup and automatic deduplication
        self.obstacle_memory = set()
        
        # Pre-compute and cache the attractive potential (only depends on goal)
        self._attractive_cache = None
        self._repulsive_cache = None
        self._initialize_caches()

    def _initialize_caches(self):
        """Initialize the potential caches."""
        num_grids = self.map.all_grids_num
        self._attractive_cache = np.zeros(num_grids)
        self._repulsive_cache = np.zeros(num_grids)
        
        # Pre-compute attractive potential (constant for a given goal)
        for vector_idx in range(num_grids):
            center_x, center_y = self.map.calculate_grid_center_xy_pos_from_vector_index(vector_idx)
            self._attractive_cache[vector_idx] = self._compute_attractive_potential(center_x, center_y)

    def clear_memory(self):
        """
        Clear the persistent obstacle memory.
        Use this when starting a new navigation session or
        when the environment has changed significantly.
        """
        self.obstacle_memory.clear()
        self._repulsive_cache.fill(0.0)
        # Update the map with just attractive potential
        self._update_total_potential()

    def _discretize_position(self, x_m, y_m):
        """
        Convert continuous position to a discretized grid-aligned position.
        This ensures consistent obstacle representation in memory.

        Returns: Tuple of (discretized_x, discretized_y) or None if outside map
        """
        # Get grid indices for this position using the correct API
        x_idx = self.map.calculate_xy_index_from_position(
            x_m, self.map.left_bottom_x_m, self.map.width_grids_num - 1)
        y_idx = self.map.calculate_xy_index_from_position(
            y_m, self.map.left_bottom_y_m, self.map.height_grids_num - 1)

        # Check if position is within map bounds
        if x_idx is None or y_idx is None:
            return None

        # Convert back to grid center position for consistent representation
        vector_idx = self.map.calculate_vector_index_from_xy_index(x_idx, y_idx)
        if vector_idx is None:
            return None

        center_x, center_y = self.map.calculate_grid_center_xy_pos_from_vector_index(vector_idx)
        return (center_x, center_y)

    def _get_affected_grid_range(self, obs_x, obs_y):
        """
        Get the range of grid indices affected by an obstacle at (obs_x, obs_y).
        Only grids within the influence radius (rho) need to be updated.
        """
        # Calculate index bounds based on influence radius
        min_x = obs_x - self.rho
        max_x = obs_x + self.rho
        min_y = obs_y - self.rho
        max_y = obs_y + self.rho
        
        # Convert to grid indices
        min_x_idx = self.map.calculate_xy_index_from_position(
            min_x, self.map.left_bottom_x_m, self.map.width_grids_num - 1)
        max_x_idx = self.map.calculate_xy_index_from_position(
            max_x, self.map.left_bottom_x_m, self.map.width_grids_num - 1)
        min_y_idx = self.map.calculate_xy_index_from_position(
            min_y, self.map.left_bottom_y_m, self.map.height_grids_num - 1)
        max_y_idx = self.map.calculate_xy_index_from_position(
            max_y, self.map.left_bottom_y_m, self.map.height_grids_num - 1)
        
        # Handle out-of-bounds cases
        if min_x_idx is None: min_x_idx = 0
        if max_x_idx is None: max_x_idx = self.map.width_grids_num - 1
        if min_y_idx is None: min_y_idx = 0
        if max_y_idx is None: max_y_idx = self.map.height_grids_num - 1
        
        return min_x_idx, max_x_idx, min_y_idx, max_y_idx

    def _add_obstacle_repulsion(self, obs_x, obs_y):
        """
        Add repulsive potential for a single obstacle to the cache.
        Only updates grid cells within the influence radius.
        """
        min_x_idx, max_x_idx, min_y_idx, max_y_idx = self._get_affected_grid_range(obs_x, obs_y)
        eps = 1e-9
        
        for x_idx in range(min_x_idx, max_x_idx + 1):
            for y_idx in range(min_y_idx, max_y_idx + 1):
                vector_idx = self.map.calculate_vector_index_from_xy_index(x_idx, y_idx)
                center_x, center_y = self.map.calculate_grid_center_xy_pos_from_xy_index(x_idx, y_idx)
                
                d = np.sqrt((center_x - obs_x)**2 + (center_y - obs_y)**2)
                if d <= self.rho:
                    u_rep = 0.5 * self.eta * (1.0/(d + eps) - 1.0/self.rho)**2
                    self._repulsive_cache[vector_idx] += u_rep

    def _update_total_potential(self):
        """Update the total potential in the grid map from caches."""
        for vector_idx in range(self.map.all_grids_num):
            total = min(self._attractive_cache[vector_idx] + self._repulsive_cache[vector_idx], 
                       self.max_cost)
            self.map.set_grid_data(vector_idx, FloatGrid(value=total))

    def update_map(self, points_x_list, points_y_list):
        """
        Function to update potential field map with persistent memory.

        New obstacle observations are incrementally added to the cached
        repulsive field, ensuring efficient updates even as the obstacle
        memory grows.

        points_x_list: List of x coordinates of point cloud
        points_y_list: List of y coordinates of point cloud
        """
        new_obstacles = []
        
        # Accumulate new obstacle observations into persistent memory
        for x, y in zip(points_x_list, points_y_list):
            discretized = self._discretize_position(x, y)
            if discretized is not None and discretized not in self.obstacle_memory:
                self.obstacle_memory.add(discretized)
                new_obstacles.append(discretized)
        
        # Only update repulsive field for NEW obstacles (incremental update)
        for obs_x, obs_y in new_obstacles:
            self._add_obstacle_repulsion(obs_x, obs_y)
        
        # Update total potential map
        self._update_total_potential()

    def _compute_attractive_potential(self, x_m, y_m):
        """
        Compute attractive potential at a specific position.
        """
        d_squared = (x_m - self.goal_x_m)**2 + (y_m - self.goal_y_m)**2
        return 0.5 * self.zeta * d_squared

    def calculate_attractive_potential(self, x_m, y_m):
        """
        Function to calculate attractive potential at a specific position
        x_m: x coordinate position[m]
        y_m: y coordinate position[m]
        Return: Attractive potential at position
        """
        return self._compute_attractive_potential(x_m, y_m)

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

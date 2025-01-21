import numpy as np
import matplotlib.pyplot as plt
import heapq
import matplotlib.animation as anm


class AStarPathPlanner:
    def __init__(self, start, goal, obstacle_parameters, resolution=0.1, weight=1.0, obstacle_clearance=0.0, robot_clearance=0.0, visualize=False, x_lim=None, y_lim=None):
        """
        Initialize the A* planner.
        Args:
            start: (x, y) tuple for start position.
            goal: (x, y) tuple for goal position.
            obstacle_parameters: List of obstacle dictionaries.
            resolution: Grid resolution in meters.
            weight: Heuristic weight for A*.
            visualize: Boolean to enable visualization during the search.
            x_lim: (min, max) tuple for x-axis range of the grid.
            y_lim: (min, max) tuple for y-axis range of the grid.
        """
        self.start = start
        self.goal = goal
        self.obstacle_parameters = obstacle_parameters
        self.resolution = resolution
        self.weight = weight
        self.visualize = visualize
        self.x_lim = x_lim
        self.y_lim = y_lim
        self.obstacle_clearance = obstacle_clearance
        self.robot_clearance = robot_clearance
        self.clearance = obstacle_clearance + robot_clearance
        self.grid, self.x_range, self.y_range = self.create_grid()
        self.mark_obstacles()

        self.explored_nodes = []


        if visualize:
            plt.figure(figsize=(10, 8))
            plt.imshow(self.grid, extent=[self.x_range[0], self.x_range[-1], self.y_range[0], self.y_range[-1]],
                    origin='lower', cmap='Greys')
            plt.plot(start[0], start[1], 'go', label="Start")  # Start point
            plt.plot(goal[0], goal[1], 'ro', label="Goal")    # Goal point
            plt.legend()
            plt.show()


    def create_grid(self):
        """Create a grid based on the specified or derived limits."""
        if self.x_lim and self.y_lim:
            x_min, x_max = self.x_lim.min_value(), self.x_lim.max_value()
            y_min, y_max = self.y_lim.min_value(), self.y_lim.max_value()
        
        x_range = np.arange(x_min, x_max, self.resolution)
        y_range = np.arange(y_min, y_max, self.resolution)
        # print("x_range: ", x_range)
        # print("y_range: ", y_range)
        grid = np.zeros((len(y_range), len(x_range)))  # Initialize grid as free space
        # print("grid element: ", grid[-10][0])
        return grid, x_range, y_range

    def mark_obstacles(self):
        """Mark obstacles and their clearance on the grid, considering rotation (yaw)."""
        for obs in self.obstacle_parameters:
            # Get obstacle parameters
            x_c = obs["x_m"]
            y_c = obs["y_m"]
            yaw = obs["yaw_rad"]
            length = obs["length_m"]
            width = obs["width_m"]

            # Calculate the clearance dimensions
            clearance_length = length + self.clearance
            clearance_width = width + self.clearance

            # Define corners for the clearance area
            clearance_corners = np.array([
                [-clearance_length, -clearance_width],
                [-clearance_length, clearance_width],
                [clearance_length, clearance_width],
                [clearance_length, -clearance_width]
            ])

            # Define corners for the actual obstacle
            obstacle_corners = np.array([
                [-length, -width],
                [-length, width],
                [length, width],
                [length, -width]
            ])

            # Apply rotation to both obstacle and clearance corners
            rotation_matrix = np.array([
                [np.cos(yaw), -np.sin(yaw)],
                [np.sin(yaw), np.cos(yaw)]
            ])
            rotated_clearance_corners = np.dot(clearance_corners, rotation_matrix.T) + np.array([x_c, y_c])
            rotated_obstacle_corners = np.dot(obstacle_corners, rotation_matrix.T) + np.array([x_c, y_c])

            # Mark the clearance area
            self._mark_area(rotated_clearance_corners, value=0.5)  # 0.5 for clearance

            # Mark the actual obstacle area
            self._mark_area(rotated_obstacle_corners, value=1.0)  # 1.0 for obstacles

    def _point_in_polygon(self, x, y, corners):
        """
        Check if a point (x, y) is inside a polygon defined by corners.
        Args:
            x: X-coordinate of the point.
            y: Y-coordinate of the point.
            corners: Array of polygon corners in global coordinates.
        Returns:
            True if the point is inside the polygon, False otherwise.
        """
        n = len(corners)
        inside = False
        px, py = x, y
        for i in range(n):
            x1, y1 = corners[i]
            x2, y2 = corners[(i + 1) % n]
            if ((y1 > py) != (y2 > py)) and \
            (px < (x2 - x1) * (py - y1) / (y2 - y1 + 1e-6) + x1):
                inside = not inside
        return inside


    def _mark_area(self, corners, value):
        """
        Mark a rectangular area on the grid based on the given rotated corners.
        Args:
            corners: The rotated corners of the area in global coordinates.
            value: The value to mark in the grid (e.g., 0.5 for clearance, 1.0 for obstacles).
        """
        # Get the bounding box of the corners
        x_min = max(0, int((min(corners[:, 0]) - self.x_range[0]) / self.resolution))
        x_max = min(self.grid.shape[1], int((max(corners[:, 0]) - self.x_range[0]) / self.resolution))
        y_min = max(0, int((min(corners[:, 1]) - self.y_range[0]) / self.resolution))
        y_max = min(self.grid.shape[0], int((max(corners[:, 1]) - self.y_range[0]) / self.resolution))

        # Iterate through the grid cells in the bounding box
        for x in range(x_min, x_max):
            for y in range(y_min, y_max):
                # Get the center of the current cell
                cell_x = self.x_range[0] + x * self.resolution + self.resolution / 2
                cell_y = self.y_range[0] + y * self.resolution + self.resolution / 2

                # Check if the cell center is inside the rotated polygon
                if self._point_in_polygon(cell_x, cell_y, corners):
                    self.grid[y, x] = max(self.grid[y, x], value)  # Mark the cell


    def heuristic(self, a, b):
        return self.weight * (abs(a[0] - b[0]) + abs(a[1] - b[1]))

    def is_valid(self, x, y):
        """
        Check if a grid cell is within bounds and not an obstacle.
        Converts world coordinates to grid indices, accounting for negative min values.
        """
        # Check if indices are within bounds and not an obstacle
        return (0 <= x < self.grid.shape[1] and 
                0 <= y < self.grid.shape[0] and 
                self.grid[y, x] == 0)

    def search(self):
        start_idx = (int((self.start[0] - self.x_range[0]) / self.resolution),
                     int((self.start[1] - self.y_range[0]) / self.resolution))
        goal_idx = (int((self.goal[0] - self.x_range[0]) / self.resolution),
                    int((self.goal[1] - self.y_range[0]) / self.resolution))

        open_list = []
        heapq.heappush(open_list, (0, start_idx))
        came_from = {}
        cost_so_far = {start_idx: 0}

        print(f"Start: {start_idx}, Goal: {goal_idx}")
        while open_list:
            _, current = heapq.heappop(open_list)
            self.explored_nodes.append(current)
            if current == goal_idx:
                print(f"Goal found at: {current}")
                return self.reconstruct_path(came_from, start_idx, goal_idx)

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1),(1, 1), (-1, -1), (1, -1), (-1, 1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                # print(f"Neighbor: {neighbor}")
                if self.is_valid(neighbor[0], neighbor[1]):
                    new_cost = cost_so_far[current] + 1
                    if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                        cost_so_far[neighbor] = new_cost
                        priority = new_cost + self.heuristic(neighbor, goal_idx)
                        heapq.heappush(open_list, (priority, neighbor))
                        came_from[neighbor] = current
                

        return []

    def reconstruct_path(self, came_from, start, goal):
        """
        Reconstruct the path from start to goal in world coordinates.
        Args:
            came_from: Dictionary containing the parent of each node.
            start: Start node in grid indices.
            goal: Goal node in grid indices.
        Returns:
            path: List of (x, y) tuples in world coordinates.
        """
        current = goal
        path = []
        while current != start:
            path.append(current)  # Convert grid indices to world coordinates
            current = came_from[current]
        path.append(start)  # Add the start node in world coordinates
        return path[::-1]  # Reverse the path


    def _grid_to_world(self, grid_node):
        """
        Convert grid indices to world coordinates.
        Args:
            grid_node: (grid_x, grid_y) tuple in grid indices.
        Returns:
            (world_x, world_y): Corresponding world coordinates.
        """
        grid_x, grid_y = grid_node
        world_x = self.x_range[0] + grid_x * self.resolution
        world_y = self.y_range[0] + grid_y * self.resolution
        return (world_x, world_y)

    def make_sparse_path(self, path, num_points=20):
        """
        Make the path sparse for use with CubicSplineCourse.
        Args:
            path: Full path as a list of (x, y) tuples in world coordinates.
            num_points: Number of points to include in the sparse path.
        Returns:
            sparse_path: A sparse path with evenly spaced world coordinates.
        """
        if len(path) <= num_points:
            # If the path already has fewer points than num_points, return as-is
            return path

        # Use linear spacing to select points
        indices = np.linspace(0, len(path) - 1, num_points, dtype=int)
        sparse_path = [self._grid_to_world(path[i]) for i in indices]
        return sparse_path
    
    def visualize_search(self, path, gif_name=None):
        print(f"Exploring {len(self.explored_nodes)} nodes.")
        if not self.explored_nodes:
            print("Error: No explored nodes. Ensure search() is executed before visualize_search().")
            return


        figure = plt.figure(figsize=(10, 8))
        axes = figure.add_subplot(111)
        axes.set_aspect("equal")
        axes.set_xlabel("X [m]", fontsize=15)
        axes.set_ylabel("Y [m]", fontsize=15)

        self.anime = anm.FuncAnimation(
            figure,
            self.update_frame,
            fargs=(axes, path),
            frames=len(self.explored_nodes) + len(path),  # Include frames for the path
            interval=50,
            repeat=False,
        )

        if gif_name:
            try:
                self.anime.save(gif_name, writer="pillow")
            except Exception as e:
                print(f"Error saving animation: {e}")
        plt.show()


    def update_frame(self, i, axes, path):
        """
        Update frame for visualization using cell filling, including path reconstruction.
        Args:
            i: Current frame index.
            axes: Matplotlib axes to draw on.
            path: The reconstructed path to draw after exploration.
        """
        # Exploration phase
        if i < len(self.explored_nodes):
            # Mark the current node as explored
            node = self.explored_nodes[i]
            grid_x = int(node[0])
            grid_y = int(node[1])
            self.grid[grid_y, grid_x] = 0.5  # Set a value to represent explored nodes

        # Path reconstruction phase
        else:
            path_index = i - len(self.explored_nodes)
            if path_index < len(path):
                node = path[path_index]
                grid_x = int(node[0])
                grid_y = int(node[1])
                self.grid[grid_y, grid_x] = 0.75  # Set a value to represent the path

        # Clear the axes and redraw the updated grid
        axes.clear()
        axes.imshow(self.grid, extent=[self.x_range[0], self.x_range[-1], self.y_range[0], self.y_range[-1]],
                    origin='lower', cmap='coolwarm', alpha=0.8)
        axes.plot(self.start[0], self.start[1], 'go', label="Start")
        axes.plot(self.goal[0], self.goal[1], 'ro', label="Goal")
        axes.legend()



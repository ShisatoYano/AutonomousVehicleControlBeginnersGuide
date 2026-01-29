"""
astar_bidirectional_path_planner.py

Author: Auto-generated

This module implements the Bidirectional A* algorithm for path planning.
Bidirectional A* runs two A* searches simultaneously - one from start to goal
and one from goal to start. When the two searches meet, the paths are combined.
This approach can be more efficient than standard A* in many cases.
"""

import numpy as np
import matplotlib.pyplot as plt
import heapq
import matplotlib.animation as anm
import sys
from pathlib import Path
from matplotlib.colors import ListedColormap

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"
relative_simulations = "/../../../simulations/"


sys.path.append(abs_dir_path + relative_path + "visualization")
sys.path.append(abs_dir_path + relative_path + "state")
sys.path.append(abs_dir_path + relative_path + "obstacle")
sys.path.append(abs_dir_path + relative_path + "plan/astar")
sys.path.append(abs_dir_path + relative_path + "mapping/grid")


from state import State
from obstacle import Obstacle
from obstacle_list import ObstacleList
from binary_occupancy_grid import BinaryOccupancyGrid
from min_max import MinMax
import json


class AStarBidirectionalPathPlanner:
    def __init__(self, start, goal, map_file, weight=1.0, x_lim=None, y_lim=None, path_filename=None, gif_name=None):
        """
        Initialize the Bidirectional A* planner.
        Args:
            start: (x, y) tuple for start position.
            goal: (x, y) tuple for goal position.
            map_file: Path to the grid map file.
            weight: Heuristic weight for A*.
            x_lim: MinMax object for x-axis range of the grid.
            y_lim: MinMax object for y-axis range of the grid.
            path_filename: Path to save the generated path.
            gif_name: Path to save the animation as a GIF.
        """
        self.start = start
        self.goal = goal
        self.weight = weight
        self.explored_nodes = []
        self.grid = self.load_grid_from_file(map_file)
        x_min, x_max = x_lim.min_value(), x_lim.max_value()
        y_min, y_max = y_lim.min_value(), y_lim.max_value()
        self.resolution = (x_max - x_min) / self.grid.shape[1]  # Width of each cell
        self.x_range = np.arange(x_min, x_max, self.resolution)
        self.y_range = np.arange(y_min, y_max, self.resolution)
        self.x_min, self.x_max = x_min, x_max
        self.y_min, self.y_max = y_min, y_max
        self.path = []
        self.path_filename = path_filename
        self.meeting_point = None
        self.search()
        self.visualize_search(gif_name)

    def load_grid_from_file(self, file_path):
        """
        Load a grid from a file and convert it to a numpy array.
        Args:
            file_path: Path to the file containing the grid data.
        Returns:
            grid: A numpy array representing the grid.
        """
        file_extension = Path(file_path).suffix

        if file_extension == '.npy':
            grid = np.load(file_path)
        elif file_extension == '.png':
            grid = plt.imread(file_path)
            if grid.ndim == 3:  # If the image has color channels, convert to grayscale
                grid = np.mean(grid, axis=2)
            grid = (grid > 0.5).astype(int)  # Binarize the image
        elif file_extension == '.json':
            with open(file_path, 'r') as f:
                grid_data = json.load(f)
            grid = np.array(grid_data)
        else:
            raise ValueError(f"Unsupported file format: {file_extension}")

        return grid

    def heuristic(self, a, b):
        """
        Calculate Manhattan distance heuristic.
        Args:
            a: (x, y) tuple for first position.
            b: (x, y) tuple for second position.
        Returns:
            Heuristic distance.
        """
        return self.weight * (abs(a[0] - b[0]) + abs(a[1] - b[1]))

    def is_valid(self, x, y):
        """
        Check if a grid cell is within bounds and not an obstacle.
        Args:
            x: Grid x index.
            y: Grid y index.
        Returns:
            True if valid, False otherwise.
        """
        return (0 <= x < self.grid.shape[1] and
                0 <= y < self.grid.shape[0] and
                self.grid[y, x] == 0)

    def get_neighbors(self, node):
        """
        Get valid neighbors of a node (8-connected).
        Args:
            node: (x, y) tuple in grid indices.
        Returns:
            List of valid neighbor nodes.
        """
        neighbors = []
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (1, 1), (-1, -1), (1, -1), (-1, 1)]:
            neighbor = (node[0] + dx, node[1] + dy)
            if self.is_valid(neighbor[0], neighbor[1]):
                neighbors.append(neighbor)
        return neighbors

    def get_cost(self, from_node, to_node):
        """
        Calculate cost between two adjacent nodes.
        Args:
            from_node: (x, y) tuple in grid indices.
            to_node: (x, y) tuple in grid indices.
        Returns:
            Cost to move from from_node to to_node.
        """
        dx = abs(to_node[0] - from_node[0])
        dy = abs(to_node[1] - from_node[1])
        if dx == 1 and dy == 1:
            return np.sqrt(2)  # Diagonal movement
        return 1.0  # Orthogonal movement

    def search(self):
        """
        Bidirectional A* algorithm to find a path from start to goal.
        Runs two A* searches simultaneously and combines paths when they meet.
        """
        start_idx = (int((self.start[0] - self.x_range[0]) / self.resolution),
                     int((self.start[1] - self.y_range[0]) / self.resolution))
        goal_idx = (int((self.goal[0] - self.x_range[0]) / self.resolution),
                    int((self.goal[1] - self.y_range[0]) / self.resolution))

        print(f"Start: {start_idx}, Goal: {goal_idx}")

        # Forward search: from start to goal
        forward_open = []
        forward_came_from = {}
        forward_cost_so_far = {start_idx: 0}
        forward_closed = set()
        h0_forward = self.heuristic(start_idx, goal_idx)
        heapq.heappush(forward_open, (h0_forward, start_idx))

        # Backward search: from goal to start
        backward_open = []
        backward_came_from = {}
        backward_cost_so_far = {goal_idx: 0}
        backward_closed = set()
        h0_backward = self.heuristic(goal_idx, start_idx)
        heapq.heappush(backward_open, (h0_backward, goal_idx))

        # Alternate between forward and backward searches
        while forward_open and backward_open:
            # Forward search step
            if forward_open:
                _, forward_current = heapq.heappop(forward_open)
                
                if forward_current in forward_closed:
                    continue
                    
                forward_closed.add(forward_current)
                self.explored_nodes.append(forward_current)

                # Check if forward search reached a node in backward closed set
                if forward_current in backward_closed:
                    print(f"Paths met at: {forward_current}")
                    self.meeting_point = forward_current
                    self.path = self.combine_paths(
                        forward_came_from, backward_came_from,
                        start_idx, goal_idx, forward_current
                    )
                    sparse_path = self.make_sparse_path(self.path)
                    self.save_path(sparse_path, self.path_filename)
                    return

                # Expand forward search
                for neighbor in self.get_neighbors(forward_current):
                    if neighbor in forward_closed:
                        continue
                    
                    new_cost = forward_cost_so_far[forward_current] + self.get_cost(forward_current, neighbor)
                    
                    if neighbor not in forward_cost_so_far or new_cost < forward_cost_so_far[neighbor]:
                        forward_cost_so_far[neighbor] = new_cost
                        priority = new_cost + self.heuristic(neighbor, goal_idx)
                        heapq.heappush(forward_open, (priority, neighbor))
                        forward_came_from[neighbor] = forward_current

                        # Check if this neighbor is in backward closed set
                        if neighbor in backward_closed:
                            print(f"Paths met at: {neighbor}")
                            self.meeting_point = neighbor
                            self.path = self.combine_paths(
                                forward_came_from, backward_came_from,
                                start_idx, goal_idx, neighbor
                            )
                            sparse_path = self.make_sparse_path(self.path)
                            self.save_path(sparse_path, self.path_filename)
                            return

            # Backward search step
            if backward_open:
                _, backward_current = heapq.heappop(backward_open)
                
                if backward_current in backward_closed:
                    continue
                    
                backward_closed.add(backward_current)
                self.explored_nodes.append(backward_current)

                # Check if backward search reached a node in forward closed set
                if backward_current in forward_closed:
                    print(f"Paths met at: {backward_current}")
                    self.meeting_point = backward_current
                    self.path = self.combine_paths(
                        forward_came_from, backward_came_from,
                        start_idx, goal_idx, backward_current
                    )
                    sparse_path = self.make_sparse_path(self.path)
                    self.save_path(sparse_path, self.path_filename)
                    return

                # Expand backward search
                for neighbor in self.get_neighbors(backward_current):
                    if neighbor in backward_closed:
                        continue
                    
                    new_cost = backward_cost_so_far[backward_current] + self.get_cost(backward_current, neighbor)
                    
                    if neighbor not in backward_cost_so_far or new_cost < backward_cost_so_far[neighbor]:
                        backward_cost_so_far[neighbor] = new_cost
                        priority = new_cost + self.heuristic(neighbor, start_idx)
                        heapq.heappush(backward_open, (priority, neighbor))
                        backward_came_from[neighbor] = backward_current

                        # Check if this neighbor is in forward closed set
                        if neighbor in forward_closed:
                            print(f"Paths met at: {neighbor}")
                            self.meeting_point = neighbor
                            self.path = self.combine_paths(
                                forward_came_from, backward_came_from,
                                start_idx, goal_idx, neighbor
                            )
                            sparse_path = self.make_sparse_path(self.path)
                            self.save_path(sparse_path, self.path_filename)
                            return

        print("Goal not found. No path exists.")
        return []

    def combine_paths(self, forward_came_from, backward_came_from, start, goal, meeting_point):
        """
        Combine paths from forward and backward searches.
        Args:
            forward_came_from: Parent map from forward search.
            backward_came_from: Parent map from backward search.
            start: Start node in grid indices.
            goal: Goal node in grid indices.
            meeting_point: Node where the two searches met.
        Returns:
            Combined path from start to goal in grid indices.
        """
        # Path from start to meeting point
        forward_path = []
        current = meeting_point
        while current != start:
            forward_path.append(current)
            if current not in forward_came_from:
                # This shouldn't happen, but handle edge case
                break
            current = forward_came_from[current]
        forward_path.append(start)
        forward_path = forward_path[::-1]  # Reverse to get start -> meeting_point

        # Path from meeting point to goal (backward search goes goal -> meeting_point)
        backward_path = []
        current = meeting_point
        while current != goal:
            backward_path.append(current)
            if current not in backward_came_from:
                # This shouldn't happen, but handle edge case
                break
            current = backward_came_from[current]
        backward_path.append(goal)

        # Combine paths (meeting point appears once)
        combined_path = forward_path + backward_path[1:]  # Skip first element to avoid duplicate
        return combined_path

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
            path: Full path as a list of (x, y) tuples in grid indices.
            num_points: Number of points to include in the sparse path.
        Returns:
            sparse_path: A sparse path with evenly spaced world coordinates.
        """
        if len(path) <= num_points:
            # If the path already has fewer points than num_points, return as-is
            return [self._grid_to_world(p) for p in path]

        # Use linear spacing to select points
        indices = np.linspace(0, len(path) - 1, num_points, dtype=int)
        sparse_path = [self._grid_to_world(path[i]) for i in indices]
        return sparse_path

    def save_path(self, path, filename):
        """Save path to a json file."""
        if filename is None:
            return
        if not Path(filename).exists():
            Path(filename).touch()
        path = [node for node in path]
        with open(filename, "w") as f:
            json.dump(path, f)

    def visualize_search(self, gif_name=None):
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
            fargs=(axes, self.path),
            frames=len(self.explored_nodes) + len(self.path),  # Include frames for the path
            interval=50,
            repeat=False,
        )

        if gif_name is not None:
            try:
                print("Saving animation...")
                self.anime.save(gif_name, writer="pillow")
                print("Animation saved successfully.")
            except Exception as e:
                print(f"Error saving animation: {e}")
        else:
            plt.show()

        # clear existing plot and close existing figure
        plt.clf()
        plt.close()

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
            self.grid[grid_y, grid_x] = 0.25  # Set a value to represent explored nodes

        # Path reconstruction phase
        else:
            path_index = i - len(self.explored_nodes)
            if path_index < len(path):
                node = path[path_index]
                grid_x = int(node[0])
                grid_y = int(node[1])
                self.grid[grid_y, grid_x] = 0.5  # Set a value to represent the path

        # Clear the axes and redraw the updated grid
        axes.clear()

        # Define RGB colors for each grid value
        # Colors in the format [R, G, B], where values are in the range [0, 1]
        colors = [
            [1.0, 1.0, 1.0],  # Free space (white)
            [0.4, 0.8, 1.0],  # Explored nodes (light blue)
            [0.0, 1.0, 0.0],  # Path (green)
            [0.5, 0.5, 0.5],  # Clearance space (gray)
            [0.0, 0.0, 0.0],  # Obstacles (black)
        ]

        # Create a colormap
        custom_cmap = ListedColormap(colors)

        axes.imshow(self.grid, extent=[self.x_range[0], self.x_range[-1], self.y_range[0], self.y_range[-1]],
                    origin='lower', cmap=custom_cmap, alpha=0.8)
        axes.plot(self.start[0], self.start[1], 'go', label="Start")
        axes.plot(self.goal[0], self.goal[1], 'ro', label="Goal")
        if self.meeting_point:
            meeting_world = self._grid_to_world(self.meeting_point)
            axes.plot(meeting_world[0], meeting_world[1], 'mo', markersize=8, label="Meeting Point")
        axes.legend()


if __name__ == "__main__":

    # The path to the map file where the planner will search for a path
    map_file = "map.json"
    # Define the path file to save the path that is generated by the planner
    path_file = "path.json"
    # Visualize the search process and save the gif
    gif_path = "astar_bidirectional_search.gif"

    x_lim, y_lim = MinMax(-5, 55), MinMax(-20, 25)

    # Define the start and goal positions
    start = (0, 0)
    goal = (50, -10)

    # Create the Bidirectional A* planner
    planner = AStarBidirectionalPathPlanner(start, goal, map_file, weight=5.0, x_lim=x_lim, y_lim=y_lim, path_filename=path_file, gif_name=gif_path)

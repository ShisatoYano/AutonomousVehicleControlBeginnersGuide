"""
rrt_star_bidirectional_path_planner.py

Author: Auto-generated

This module implements the Bidirectional RRT* algorithm for path planning.
Bidirectional RRT* runs two RRT* searches simultaneously - one from start to goal
and one from goal to start. When the two trees meet, the paths are combined.
This approach combines the benefits of RRT* optimization with bidirectional search efficiency.
"""

import numpy as np
import matplotlib.pyplot as plt
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


class RrtStarBidirectionalPathPlanner:
    def __init__(self, start, goal, map_file, x_lim=None, y_lim=None, path_filename=None, gif_name=None, max_iterations=5000, step_size=0.5, goal_sample_rate=0.05, rewire_radius=None):
        """
        Initialize the Bidirectional RRT* planner.
        Args:
            start: (x, y) tuple for start position in world coordinates.
            goal: (x, y) tuple for goal position in world coordinates.
            map_file: Path to the grid map file.
            x_lim: MinMax object for x-axis range of the grid.
            y_lim: MinMax object for y-axis range of the grid.
            path_filename: Path to save the generated path.
            gif_name: Path to save the animation as a GIF.
            max_iterations: Maximum number of iterations for RRT*.
            step_size: Maximum distance to extend the tree in each step.
            goal_sample_rate: Probability of sampling the goal position.
            rewire_radius: Radius for finding nearby nodes for rewiring. If None, uses step_size * 2.
        """
        self.start = start
        self.goal = goal
        self.explored_nodes = []
        self.tree_edges = []  # Store edges for visualization
        self.grid = self.load_grid_from_file(map_file)
        x_min, x_max = x_lim.min_value(), x_lim.max_value()
        y_min, y_max = y_lim.min_value(), y_lim.max_value()
        self.resolution = (x_max - x_min) / self.grid.shape[1]
        self.x_range = np.arange(x_min, x_max, self.resolution)
        self.y_range = np.arange(y_min, y_max, self.resolution)
        self.x_min, self.x_max = x_min, x_max
        self.y_min, self.y_max = y_min, y_max
        self.path = []
        self.path_filename = path_filename
        self.max_iterations = max_iterations
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.rewire_radius = rewire_radius if rewire_radius is not None else step_size * 2.0
        self.meeting_point = None
        self.start_idx = self._world_to_grid(self.start)
        self.goal_idx = self._world_to_grid(self.goal)

        self.search()
        self.visualize_search(gif_name)

    def load_grid_from_file(self, file_path):
        """Load a grid from a file and convert it to a numpy array."""
        file_extension = Path(file_path).suffix

        if file_extension == '.npy':
            grid = np.load(file_path)
        elif file_extension == '.png':
            grid = plt.imread(file_path)
            if grid.ndim == 3:
                grid = np.mean(grid, axis=2)
            grid = (grid > 0.5).astype(int)
        elif file_extension == '.json':
            with open(file_path, 'r') as f:
                grid_data = json.load(f)
            grid = np.array(grid_data)
        else:
            raise ValueError(f"Unsupported file format: {file_extension}")

        return grid

    def _world_to_grid(self, world_point):
        """Convert world coordinates to grid indices."""
        grid_x = int((world_point[0] - self.x_range[0]) / self.resolution)
        grid_y = int((world_point[1] - self.y_range[0]) / self.resolution)
        return (grid_x, grid_y)

    def is_valid(self, x, y):
        """Check if a grid cell is within bounds and not an obstacle."""
        return (0 <= x < self.grid.shape[1] and
                0 <= y < self.grid.shape[0] and
                self.grid[y, x] == 0)

    def line_collision_check(self, point1, point2, num_samples=20):
        """Check if the line segment between two points collides with obstacles."""
        for i in range(num_samples + 1):
            t = i / num_samples
            x = point1[0] + t * (point2[0] - point1[0])
            y = point1[1] + t * (point2[1] - point1[1])
            grid_x, grid_y = self._world_to_grid((x, y))
            if not self.is_valid(grid_x, grid_y):
                return False
        return True

    def get_nearest_node(self, nodes, random_point):
        """Find the nearest node in the tree to a random point."""
        distances = [np.linalg.norm(np.array(node) - np.array(random_point)) for node in nodes]
        nearest_idx = np.argmin(distances)
        return nodes[nearest_idx], nearest_idx

    def extend(self, nearest_node, random_point):
        """Extend the tree from the nearest node towards the random point."""
        direction = np.array(random_point) - np.array(nearest_node)
        distance = np.linalg.norm(direction)
        if distance < 1e-6:
            return None
        direction = direction / distance
        new_point = np.array(nearest_node) + direction * min(self.step_size, distance)
        new_point = tuple(new_point)
        return new_point

    def get_nearby_nodes(self, nodes, point, radius):
        """Find all nodes within a given radius of a point."""
        nearby = []
        for idx, node in enumerate(nodes):
            distance = np.linalg.norm(np.array(node) - np.array(point))
            if distance <= radius:
                nearby.append((node, idx))
        return nearby

    def _update_descendant_costs(self, nodes, parent, cost, node_idx):
        """Update costs of all descendants after rewiring."""
        children = [idx for idx, p_idx in parent.items() if p_idx == node_idx]
        for child_idx in children:
            if child_idx in cost and child_idx in parent and parent[child_idx] is not None:
                parent_idx = parent[child_idx]
                cost[child_idx] = cost[parent_idx] + np.linalg.norm(np.array(nodes[child_idx]) - np.array(nodes[parent_idx]))
                self._update_descendant_costs(nodes, parent, cost, child_idx)

    def _extend_tree(self, nodes, parent, cost, random_point, other_nodes, other_parent, other_cost):
        """
        Extend a tree towards a random point with RRT* optimization.
        Returns meeting node if trees meet, None otherwise.
        """
        nearest_node, nearest_idx = self.get_nearest_node(nodes, random_point)
        new_node = self.extend(nearest_node, random_point)
        
        if new_node is None:
            return None
        
        if not self.line_collision_check(nearest_node, new_node):
            return None

        # Find nearby nodes for potential rewiring
        nearby_nodes = self.get_nearby_nodes(nodes, new_node, self.rewire_radius)
        
        # Choose the best parent
        best_parent_idx = nearest_idx
        best_cost = cost[nearest_idx] + np.linalg.norm(np.array(new_node) - np.array(nearest_node))
        
        for nearby_node, nearby_idx in nearby_nodes:
            if nearby_idx == nearest_idx:
                continue
            if self.line_collision_check(nearby_node, new_node):
                new_cost = cost[nearby_idx] + np.linalg.norm(np.array(new_node) - np.array(nearby_node))
                if new_cost < best_cost:
                    best_cost = new_cost
                    best_parent_idx = nearby_idx

        # Check if new_node connects to the other tree BEFORE adding it
        other_nearby = self.get_nearby_nodes(other_nodes, new_node, self.rewire_radius)
        for other_node, other_idx in other_nearby:
            if self.line_collision_check(new_node, other_node):
                # Trees meet! Add new_node and connect to other tree
                node_idx = len(nodes)
                nodes.append(new_node)
                parent[node_idx] = best_parent_idx
                cost[node_idx] = best_cost
                self.explored_nodes.append(new_node)
                self.tree_edges.append((nodes[best_parent_idx], new_node))
                # Connect to other tree
                self.tree_edges.append((new_node, other_node))
                return new_node

        # Add the new node
        node_idx = len(nodes)
        nodes.append(new_node)
        parent[node_idx] = best_parent_idx
        cost[node_idx] = best_cost
        self.explored_nodes.append(new_node)
        self.tree_edges.append((nodes[best_parent_idx], new_node))

        # Rewire nearby nodes
        for nearby_node, nearby_idx in nearby_nodes:
            if nearby_idx == node_idx:
                continue
            if self.line_collision_check(new_node, nearby_node):
                new_cost = cost[node_idx] + np.linalg.norm(np.array(nearby_node) - np.array(new_node))
                if new_cost < cost[nearby_idx]:
                    old_parent_idx = parent[nearby_idx]
                    if old_parent_idx is not None:
                        old_edge = (nodes[old_parent_idx], nearby_node)
                        if old_edge in self.tree_edges:
                            self.tree_edges.remove(old_edge)
                    parent[nearby_idx] = node_idx
                    cost[nearby_idx] = new_cost
                    self.tree_edges.append((new_node, nearby_node))
                    self._update_descendant_costs(nodes, parent, cost, nearby_idx)

        return None

    def search(self):
        """Bidirectional RRT* algorithm to find an optimized path."""
        # Forward tree: from start to goal
        forward_nodes = [self.start]
        forward_parent = {0: None}
        forward_cost = {0: 0.0}
        
        # Backward tree: from goal to start
        backward_nodes = [self.goal]
        backward_parent = {0: None}
        backward_cost = {0: 0.0}

        print(f"Start: {self.start}, Goal: {self.goal}")

        for iteration in range(self.max_iterations):
            # Alternate between forward and backward expansion
            if iteration % 2 == 0:
                # Forward tree expansion
                if np.random.rand() < self.goal_sample_rate:
                    random_point = self.goal
                else:
                    random_point = (
                        np.random.uniform(self.x_min, self.x_max),
                        np.random.uniform(self.y_min, self.y_max)
                    )
                
                meeting_node = self._extend_tree(forward_nodes, forward_parent, forward_cost, random_point,
                                                 backward_nodes, backward_parent, backward_cost)
                if meeting_node is not None:
                    # Trees met during forward expansion
                    self.meeting_point = meeting_node
                    self.path = self._combine_paths(forward_nodes, forward_parent, forward_cost,
                                                    backward_nodes, backward_parent, backward_cost,
                                                    meeting_node)
                    sparse_path = self.make_sparse_path(self.path)
                    self.save_path(sparse_path, self.path_filename)
                    print(f"Goal reached at iteration {iteration}")
                    return
            else:
                # Backward tree expansion
                if np.random.rand() < self.goal_sample_rate:
                    random_point = self.start
                else:
                    random_point = (
                        np.random.uniform(self.x_min, self.x_max),
                        np.random.uniform(self.y_min, self.y_max)
                    )
                
                meeting_node = self._extend_tree(backward_nodes, backward_parent, backward_cost, random_point,
                                                 forward_nodes, forward_parent, forward_cost)
                if meeting_node is not None:
                    # Trees met during backward expansion
                    self.meeting_point = meeting_node
                    self.path = self._combine_paths(forward_nodes, forward_parent, forward_cost,
                                                    backward_nodes, backward_parent, backward_cost,
                                                    meeting_node)
                    sparse_path = self.make_sparse_path(self.path)
                    self.save_path(sparse_path, self.path_filename)
                    print(f"Goal reached at iteration {iteration}")
                    return

        print(f"Goal not found within {self.max_iterations} iterations.")
        # Return path to closest nodes
        if len(forward_nodes) > 1 and len(backward_nodes) > 1:
            min_dist = float('inf')
            best_forward_idx = 0
            best_backward_idx = 0
            for f_idx, f_node in enumerate(forward_nodes):
                for b_idx, b_node in enumerate(backward_nodes):
                    dist = np.linalg.norm(np.array(f_node) - np.array(b_node))
                    if dist < min_dist and self.line_collision_check(f_node, b_node):
                        min_dist = dist
                        best_forward_idx = f_idx
                        best_backward_idx = b_idx
            
            if min_dist < float('inf'):
                # Connect the two closest nodes
                forward_path = self._reconstruct_path(forward_nodes, forward_parent, best_forward_idx)
                backward_path = self._reconstruct_path(backward_nodes, backward_parent, best_backward_idx)
                self.path = forward_path + backward_path[::-1][1:]
                sparse_path = self.make_sparse_path(self.path)
                self.save_path(sparse_path, self.path_filename)

    def _combine_paths(self, forward_nodes, forward_parent, forward_cost,
                      backward_nodes, backward_parent, backward_cost, meeting_node):
        """Combine paths from forward and backward trees."""
        # Find meeting node in the tree it was added to
        if meeting_node in forward_nodes:
            forward_meeting_idx = forward_nodes.index(meeting_node)
            # Find best connection point in backward tree
            backward_nearby = self.get_nearby_nodes(backward_nodes, meeting_node, self.rewire_radius)
            best_backward_idx = 0
            best_cost = float('inf')
            for other_node, other_idx in backward_nearby:
                if self.line_collision_check(meeting_node, other_node):
                    total_cost = forward_cost[forward_meeting_idx] + backward_cost[other_idx] + \
                                np.linalg.norm(np.array(meeting_node) - np.array(other_node))
                    if total_cost < best_cost:
                        best_cost = total_cost
                        best_backward_idx = other_idx
        else:
            # Meeting node is in backward tree
            backward_meeting_idx = backward_nodes.index(meeting_node)
            # Find best connection point in forward tree
            forward_nearby = self.get_nearby_nodes(forward_nodes, meeting_node, self.rewire_radius)
            best_forward_idx = 0
            best_cost = float('inf')
            for other_node, other_idx in forward_nearby:
                if self.line_collision_check(meeting_node, other_node):
                    total_cost = forward_cost[other_idx] + backward_cost[backward_meeting_idx] + \
                                np.linalg.norm(np.array(meeting_node) - np.array(other_node))
                    if total_cost < best_cost:
                        best_cost = total_cost
                        best_forward_idx = other_idx
            forward_meeting_idx = best_forward_idx
            best_backward_idx = backward_meeting_idx
        
        forward_path = self._reconstruct_path(forward_nodes, forward_parent, forward_meeting_idx)
        backward_path = self._reconstruct_path(backward_nodes, backward_parent, best_backward_idx)
        
        # Combine paths (meeting point appears once)
        return forward_path + backward_path[::-1][1:]

    def _reconstruct_path(self, nodes, parent, goal_idx):
        """Reconstruct the path from start to goal in the tree."""
        path = []
        current_idx = goal_idx
        while current_idx is not None:
            path.append(nodes[current_idx])
            current_idx = parent.get(current_idx)
        return path[::-1]

    def make_sparse_path(self, path, num_points=20):
        """Make the path sparse for use with CubicSplineCourse."""
        if len(path) <= num_points:
            return path
        indices = np.linspace(0, len(path) - 1, num_points, dtype=int)
        sparse_path = [path[i] for i in indices]
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
            frames=len(self.explored_nodes) + len(self.path),
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

        plt.clf()
        plt.close()

    def update_frame(self, i, axes, path):
        """Update frame for visualization showing bidirectional RRT* tree growth and path."""
        axes.clear()

        colors = [
            [1.0, 1.0, 1.0],  # Free space (white)
            [0.0, 0.0, 0.0],  # Obstacles (black)
        ]
        custom_cmap = ListedColormap(colors)

        axes.imshow(self.grid, extent=[self.x_range[0], self.x_range[-1], self.y_range[0], self.y_range[-1]],
                    origin='lower', cmap=custom_cmap, alpha=0.8)

        num_edges = int((i / (len(self.tree_edges) + len(self.path))) * len(self.tree_edges))
        for j in range(min(num_edges, len(self.tree_edges))):
            edge = self.tree_edges[j]
            axes.plot([edge[0][0], edge[1][0]], [edge[0][1], edge[1][1]], 'b-', linewidth=0.5, alpha=0.5)

        if self.explored_nodes:
            explored_x = [node[0] for node in self.explored_nodes[:min(i, len(self.explored_nodes))]]
            explored_y = [node[1] for node in self.explored_nodes[:min(i, len(self.explored_nodes))]]
            axes.scatter(explored_x, explored_y, c='lightblue', s=5, alpha=0.6)

        path_start_frame = len(self.explored_nodes)
        if i >= path_start_frame and len(path) > 0:
            path_index = min(i - path_start_frame, len(path) - 1)
            path_segment = path[:path_index + 1]
            if len(path_segment) > 1:
                path_x = [node[0] for node in path_segment]
                path_y = [node[1] for node in path_segment]
                axes.plot(path_x, path_y, 'g-', linewidth=2, label="Path")

        axes.plot(self.start[0], self.start[1], 'go', markersize=10, label="Start")
        axes.plot(self.goal[0], self.goal[1], 'ro', markersize=10, label="Goal")
        if self.meeting_point:
            axes.plot(self.meeting_point[0], self.meeting_point[1], 'mo', markersize=8, label="Meeting Point")

        axes.set_xlabel("X [m]", fontsize=15)
        axes.set_ylabel("Y [m]", fontsize=15)
        axes.set_title(f"Bidirectional RRT* Path Planning - Iteration {i}", fontsize=12)
        axes.legend()
        axes.set_aspect("equal")


if __name__ == "__main__":

    map_file = "map.json"
    path_file = "path.json"
    gif_path = "rrt_star_bidirectional_search.gif"

    x_lim, y_lim = MinMax(-5, 55), MinMax(-20, 25)

    start = (0, 0)
    goal = (50, -10)

    planner = RrtStarBidirectionalPathPlanner(
        start, 
        goal, 
        map_file, 
        x_lim=x_lim, 
        y_lim=y_lim, 
        path_filename=path_file, 
        gif_name=gif_path,
        max_iterations=5000,
        step_size=0.5,
        goal_sample_rate=0.05
    )

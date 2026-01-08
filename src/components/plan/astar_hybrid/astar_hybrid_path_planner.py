import math
import numpy as np
import matplotlib.pyplot as plt
import heapq
import matplotlib.animation as anm
import numpy as np
import sys
from pathlib import Path
from matplotlib.colors import ListedColormap
from typing import Tuple

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





class AStarHybridPathPlanner:
    def __init__(self, start, goal, map_file, weight=1.0, x_lim=None, y_lim=None, path_filename=None, gif_name=None):
        self.start = start  # (x, y, yaw)
        self.goal = goal    # (x, y, yaw)
        self.weight = weight
        self.grid = self.load_grid_from_file(map_file)
        
        # Extract min values from your MinMax objects
        self.x_min = x_lim.min_value()
        self.y_min = y_lim.min_value()
        x_max = x_lim.max_value()
        
        # Calculate resolution and ranges
        self.resolution = (x_max - self.x_min) / self.grid.shape[1]
        self.x_range = np.arange(self.x_min, x_max, self.resolution)
        self.y_range = np.arange(self.y_min, self.y_min + self.grid.shape[0] * self.resolution, self.resolution)

        # Hybrid A* configuration
        self.explored_nodes = []
        self.path = []
        self.path_filename = path_filename
        
        # Execute search
        self.search()
        #self.visualize_search(gif_name)
        self.visualize_static_path()

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

    def is_valid(self, gx, gy):
        """Standard grid index check."""
        return (0 <= gx < self.grid.shape[1] and
                0 <= gy < self.grid.shape[0] and
                self.grid[gy, gx] == 0)

    def world_to_grid(self, x, y):
        """Helper to convert world (m) to grid index."""
        gx = int((x - self.x_min) / self.resolution)
        gy = int((y - self.y_min) / self.resolution)
        return gx, gy

    def heuristic(self, pos, goal):
        """Euclidean distance + weight."""
        dist = self._euclidean(pos[:2], goal[:2])
        # Add a weighted penalty for being at the wrong angle
        yaw_diff = abs(math.atan2(math.sin(pos[2]-goal[2]), math.cos(pos[2]-goal[2])))
        return self.weight * (dist + self.yaw_cost_weight * yaw_diff)

    def _euclidean(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        """Standard 2D distance. Fixed Tuple casing to avoid TypeError."""
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _wrap_angle(self, theta: float) -> float:
        """
        Wraps an angle (in radians) to the range [-pi, pi].
        Ensures that 3.15 radians becomes -3.13 radians.
        """
        return (theta + math.pi) % (2 * math.pi) - math.pi

    def world_to_grid_safe(self, x: float, y: float) -> Tuple[int, int]:
        # Use x_min and y_min established in __init__
        gx = int((x - self.x_min) / self.resolution)
        gy = int((y - self.y_min) / self.resolution)
        
        # Prevent "Index Out of Range" during animation
        gx = max(0, min(gx, self.grid.shape[1] - 1))
        gy = max(0, min(gy, self.grid.shape[0] - 1))
        return gx, gy

    def search(self):
        open_list = []
        # Priority queue stores: (f_score, (x, y, yaw))
        # Use slicing [:2] to pass only (x, y) to Euclidean function
        h0 = self._euclidean(self.start[:2], self.goal[:2]) 
        heapq.heappush(open_list, (h0, self.start))
        
        came_from = {}
        cost_so_far = {self.start: 0.0}
        closed_set = set()
        yaw_res = np.radians(10.0)
        dist_tolerance = self.resolution * 1.5
        yaw_tolerance = np.radians(30.0)

        while open_list:
            _, current = heapq.heappop(open_list)
            
            # 1. Get grid indices and discretize for closed set
            pgx, pgy = self.world_to_grid_safe(current[0], current[1])
            discrete_state = (pgx, pgy, int(current[2] / yaw_res))
            
            if discrete_state in closed_set:
                continue
            closed_set.add(discrete_state)

            # 2. Add to explored_nodes as 2-value tuple for your animator
            self.explored_nodes.append((pgx, pgy))

            # 2. Updated Goal Check: Position AND Heading
            dist_error = self._euclidean(current[:2], self.goal[:2])
            yaw_error = abs(self._wrap_angle(current[2] - self.goal[2]))

            if dist_error < dist_tolerance and yaw_error < yaw_tolerance:
                print(f"Goal Reached! Pos Error: {dist_error:.2f}, Yaw Error: {math.degrees(yaw_error):.2f}")
                world_path = self.reconstruct_path(came_from, self.start, current)
                self.path = [self.world_to_grid_safe(p[0], p[1]) for p in world_path]
                self.save_path(world_path, self.path_filename)
                return

            # 4. Expansion with Motion Primitives
            for steer in [-1.0/6.0, 0.0, 1.0/6.0]:
                L = self.resolution * 3
                next_yaw = current[2] + steer * L
                next_x = current[0] + math.cos(next_yaw) * L
                next_y = current[1] + math.sin(next_yaw) * L

                ngx, ngy = self.world_to_grid_safe(next_x, next_y)
                
                if self.is_valid(ngx, ngy):
                    neighbor = (next_x, next_y, next_yaw)
                    new_cost = cost_so_far[current] + L
                    
                    if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                        cost_so_far[neighbor] = new_cost
                        # Heuristic uses sliced neighbor[:2]
                        priority = new_cost + self.weight * self._euclidean(neighbor[:2], self.goal[:2])
                        heapq.heappush(open_list, (priority, neighbor))
                        came_from[neighbor] = current

    def reconstruct_path(self, came_from, start, goal):
        path = []
        curr = goal
        while curr != start:
            path.append(curr)
            curr = came_from[curr]
        path.append(start)
        return path[::-1]

    def _grid_to_world(self, grid_node):
        """
        Convert grid indices to world coordinates.
        Args:
            grid_node: (grid_x, grid_y) tuple in grid indices.
        Returns:
            (world_x, world_y): Corresponding world coordinates.
        """
        grid_x, grid_y = grid_node
        world_x = self.x_range[0] + grid_x *self.resolution
        world_y = self.y_range[0] + grid_y *self.resolution
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

    def save_path(self, path, filename):
        """Save path to a json file."""
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
            [0.5, 0.5, 0.5],  # Clearance space (yellow-orange)
            [0.0, 0.0, 0.0],  # Obstacles (red)
        ]

        # Create a colormap
        custom_cmap = ListedColormap(colors)


        axes.imshow(self.grid, extent=[self.x_range[0], self.x_range[-1], self.y_range[0], self.y_range[-1]],
                    origin='lower', cmap=custom_cmap, alpha=0.8)
        axes.plot(self.start[0], self.start[1], 'go', label="Start")
        axes.plot(self.goal[0], self.goal[1], 'ro', label="Goal")
        axes.legend()
    
    def visualize_static_path(self):
        """
        Plots the final planned path on the map without animation.
        Useful for quick debugging or reports.
        """
        if not self.path:
            print("No path available to visualize.")
            return

        figure = plt.figure(figsize=(10, 8))
        axes = figure.add_subplot(111)
        axes.set_aspect("equal")
        axes.set_xlabel("X [m]", fontsize=12)
        axes.set_ylabel("Y [m]", fontsize=12)

        # 1. Define the colors for the map (0: Free, 1: Obstacle)
        # Using your ListedColormap logic
        bg_cmap = ListedColormap([[1.0, 1.0, 1.0], [0.5, 0.5, 0.5]]) 

        # 2. Draw the background grid
        axes.imshow(
            self.grid, 
            extent=[self.x_range[0], self.x_range[-1], self.y_range[0], self.y_range[-1]],
            origin='lower', 
            cmap=bg_cmap, 
            alpha=0.8
        )

        # 3. Convert self.path (grid indices) back to world coordinates for plotting
        path_world = [self._grid_to_world(node) for node in self.path]
        path_x = [p[0] for p in path_world]
        path_y = [p[1] for p in path_world]

        # 4. Draw the path, Start, and Goal
        axes.plot(path_x, path_y, 'b-', linewidth=2, label="Planned Path")
        axes.plot(self.start[0], self.start[1], 'go', markersize=8, label="Start")
        axes.plot(self.goal[0], self.goal[1], 'ro', markersize=8, label="Goal")

        axes.legend(loc="upper left")
        axes.grid(True, linestyle="--", alpha=0.5)
        
        plt.title("Hybrid A* Static Path Result")
        plt.show()



if __name__ == "__main__":

 

    # The path to the map file where the planner will search for a path
    map_file = "map.json"
    # Define the path file to save the path that is generated by the planner
    path_file = "astar_hybrid_path.json"
    # Visualize the search process and save the gif
    gif_path = "astar_hybrid_search2.gif"
    x_lim, y_lim = MinMax(-5, 55), MinMax(-20, 25)

    # Define the start and goal positions
    start = (0, 0, math.radians(0.0))
    goal = (50, -10, math.radians(90.0))

    # Create the A* planner

    planner = AStarHybridPathPlanner(start, goal, map_file, weight=5.0, x_lim=x_lim, y_lim=y_lim, path_filename=path_file, gif_name=gif_path)
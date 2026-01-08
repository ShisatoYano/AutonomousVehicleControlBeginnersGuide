import math
import numpy as np
import matplotlib.pyplot as plt
import heapq
import matplotlib.animation as anm
import numpy as np
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





class AStarHybridPathPlanner:
    def __init__(self, start, goal, map_file, weight=1.0, x_lim=None, y_lim=None, path_filename=None, gif_name=None):
        # Configuration for Hybrid A*
        self.R_min = 6.0          # Minimum turning radius
        self.step_size = 1.0      # Distance per simulation step
        self.n_steer = 3          # Number of steering commands [-max, 0, max]
        self.yaw_res = np.radians(5.0) # Discrete yaw for 'visited' check
        
        self.start = start # (x, y, yaw)
        self.goal = goal   # (x, y, yaw)
        self.weight = weight
        self.explored_nodes = []
        self.grid = self.load_grid_from_file(map_file)
        
        x_min, x_max = x_lim.min_value(), x_lim.max_value()
        y_min, y_max = y_lim.min_value(), y_lim.max_value()
        self.resolution = (x_max - x_min) / self.grid.shape[1]
        self.x_range = np.arange(x_min, x_max, self.resolution)
        self.y_range = np.arange(y_min, y_max, self.resolution)
        
        self.path = []
        self.path_filename = path_filename
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
        return self.weight * (abs(a[0] - b[0]) + abs(a[1] - b[1]))

    def is_valid(self, x, y):
        gx = int((x - self.x_range[0]) / self.resolution)
        gy = int((y - self.y_range[0]) / self.resolution)
        return (0 <= gx < self.grid.shape[1] and 0 <= gy < self.grid.shape[0] and self.grid[gy, gx] == 0)

    def get_neighbors(self, current_state):
        neighbors = []
        x, y, yaw = current_state
        # Steering options: Full Left, Straight, Full Right
        steer_inputs = [-1/self.R_min, 0, 1/self.R_min]
        
        for kappa in steer_inputs:
            # Simulate movement (Euler Integration)
            new_yaw = yaw + kappa * self.step_size * 5 # 5 steps per primitive
            new_x = x + math.cos(new_yaw) * self.step_size * 5
            new_y = y + math.sin(new_yaw) * self.step_size * 5
            
            if self.is_valid(new_x, new_y):
                neighbors.append((new_x, new_y, new_yaw))
        return neighbors

    def search(self):
        # Priority Queue: (priority, current_state)
        # state is (x, y, yaw)
        open_list = []
        heapq.heappush(open_list, (0, self.start))
        
        came_from = {}
        cost_so_far = {self.start: 0}
        
        # In Hybrid A*, 'closed' set uses discretized (x, y, yaw)
        closed_set = set()

        while open_list:
            _, current = heapq.heappop(open_list)
            
            # Discretize for visited check
            discrete_state = (int(current[0]/self.resolution), 
                              int(current[1]/self.resolution), 
                              int(current[2]/self.yaw_res))
            
            if discrete_state in closed_set: continue
            closed_set.add(discrete_state)
            self.explored_nodes.append(current)

            # Goal check with tolerance
            if math.sqrt((current[0]-self.goal[0])**2 + (current[1]-self.goal[1])**2) < self.resolution:
                print("Goal Reached!")
                self.path = self.reconstruct_path(came_from, self.start, current)
                self.save_path(self.path, self.path_filename)
                return

            for neighbor in self.get_neighbors(current):
                new_cost = cost_so_far[current] + self.step_size * 5
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self.heuristic(neighbor, self.goal)
                    heapq.heappush(open_list, (priority, neighbor))
                    came_from[neighbor] = current

    def reconstruct_path(self, came_from, start, goal):
        current = goal
        path = []
        while current != start:
            path.append(current)
            current = came_from[current]
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
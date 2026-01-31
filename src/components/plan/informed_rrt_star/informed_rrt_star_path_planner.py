"""
informed_rrt_star_path_planner.py

Author: Rajat Arora
Informed RRT* (Informed Rapidly-exploring Random Tree Star) path planner.
"""

import matplotlib

from matplotlib.patches import Ellipse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as anm
from matplotlib.animation import PillowWriter
from matplotlib.colors import ListedColormap
from pathlib import Path
import json
import sys

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"
sys.path.append(abs_dir_path + relative_path + "mapping/grid")

class InformedRrtStarPathPlanner:
    def __init__(
        self,
        start,
        goal,
        map_file,
        x_lim=None,
        y_lim=None,
        path_filename=None,
        gif_name=None,
        max_iterations=5000,
        step_size=0.5,
        goal_sample_rate=0.05,
        visualize_live=False,
    ):

        if visualize_live:
            matplotlib.use("TkAgg")
        else:
            matplotlib.use("Agg")

        np.random.seed(42)

        self.start = start
        self.goal = goal
        self.path_filename = path_filename
        self.max_iterations = max_iterations
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.visualize_live = visualize_live

        self.explored_nodes = []
        self.tree_edges = []
        self.path = []

        self.grid = self.load_grid_from_file(map_file)

        x_min, x_max = x_lim.min_value(), x_lim.max_value()
        y_min, y_max = y_lim.min_value(), y_lim.max_value()

        self.resolution = (x_max - x_min) / self.grid.shape[1]
        self.x_range = np.arange(x_min, x_max, self.resolution)
        self.y_range = np.arange(y_min, y_max, self.resolution)

        self.x_min, self.x_max = x_min, x_max
        self.y_min, self.y_max = y_min, y_max

        self.search()

        if not self.visualize_live:
            self.visualize_search(gif_name, frame_skip=5)

    def load_grid_from_file(self, file_path):
        with open(file_path, "r") as f:
            return np.array(json.load(f))

    def _world_to_grid(self, p):
        gx = int((p[0] - self.x_range[0]) / self.resolution)
        gy = int((p[1] - self.y_range[0]) / self.resolution)
        return gx, gy

    def is_valid(self, x, y):
        return (
            0 <= x < self.grid.shape[1]
            and 0 <= y < self.grid.shape[0]
            and self.grid[y, x] == 0
        )

    def line_collision_check(self, p1, p2, samples=20):
        for i in range(samples + 1):
            t = i / samples
            x = p1[0] + t * (p2[0] - p1[0])
            y = p1[1] + t * (p2[1] - p1[1])
            gx, gy = self._world_to_grid((x, y))
            if not self.is_valid(gx, gy):
                return False
        return True

    def get_nearest_node(self, nodes, p):
        return nodes[int(np.argmin([np.linalg.norm(np.array(n) - p) for n in nodes]))]

    def get_near_nodes(self, nodes, new_node, radius):
        return [
            i for i, n in enumerate(nodes)
            if np.linalg.norm(np.array(n) - np.array(new_node)) <= radius
        ]

    def get_rewire_radius(self, n):
        return 30.0 * np.sqrt(np.log(n) / n)

    def extend(self, nearest, rnd):
        d = np.array(rnd) - np.array(nearest)
        dist = np.linalg.norm(d)
        if dist < 1e-6:
            return None
        return tuple(np.array(nearest) + d / dist * min(self.step_size, dist))


    def sample_informed(self, c_best):
        c_min = np.linalg.norm(np.array(self.start) - np.array(self.goal))
        if c_best < float('inf'):
           
            a = c_best / 2.0
            b = np.sqrt(max(0, c_best**2 - c_min**2)) / 2.0
            theta = np.arctan2(self.goal[1] - self.start[1], self.goal[0] - self.start[0])
            centre = (np.array(self.start) + np.array(self.goal)) / 2.0

            cos_theta, sin_theta = np.cos(theta), np.sin(theta)
            C = np.array([[cos_theta, -sin_theta], 
                          [sin_theta, cos_theta]])

            r = np.sqrt(np.random.uniform(0, 1))
            phi = np.random.uniform(0, 2 * np.pi)
            x_ball = np.array([[r * a * np.cos(phi)], [r * b * np.sin(phi)]])
            
            x_rand = C @ x_ball + centre.reshape(2,1)
            
            return (x_rand[0,0], x_rand[1,0])

        else:
            return (
                np.random.uniform(self.x_min, self.x_max),
                np.random.uniform(self.y_min, self.y_max),
            )
        

    def search(self):
        nodes = [self.start]
        parent = {0: None} 
        cost = {0: 0.0} 
        goal_idx = None 
        self.history = []       
        self.cost_history = []  
        self.path_history = []  

        if self.visualize_live:
            plt.ion()
            self.fig, self.ax = plt.subplots(figsize=(10, 8))
            self.cmap = ListedColormap([[1, 1, 1], [0, 0, 0]])

        for iteration in range(self.max_iterations):
            
            # Sample point with informed sampling
            c_best = cost[goal_idx] if goal_idx is not None else float('inf')
            rnd = self.sample_informed(c_best)

            nearest = self.get_nearest_node(nodes, rnd)
            nearest_idx = nodes.index(nearest)
            new_node = self.extend(nearest, rnd)


            if new_node and self.line_collision_check(nearest, new_node):
                
                radius = self.get_rewire_radius(len(nodes))
                near_indices = self.get_near_nodes(nodes, new_node, radius)

                if goal_idx != None and nearest_idx == goal_idx:
                    continue

                # Optimization1: Choose best parent among near nodes
                min_cost = cost[nearest_idx] + np.linalg.norm(np.array(new_node) - np.array(nearest))
                best_parent_idx = nearest_idx
                for near_idx in near_indices:
                    near_node = nodes[near_idx]
                    new_cost = cost[near_idx] + np.linalg.norm(np.array(new_node) - np.array(near_node))
                    if new_cost < min_cost:
                        if self.line_collision_check(near_node, new_node):
                            min_cost = new_cost
                            best_parent_idx = near_idx


                nodes.append(new_node)
                new_idx = len(nodes) - 1
                parent[new_idx] = best_parent_idx
                cost[new_idx] = min_cost
                self.tree_edges.append((nodes[best_parent_idx], new_node))

                # Optimization 2: Rewire nearby nodes to find better paths
                for near_idx in near_indices:
                    near_node = nodes[near_idx]
                    new_cost = cost[new_idx] + np.linalg.norm(np.array(near_node) - np.array(new_node))
                    if new_cost < cost[near_idx]:
                        if self.line_collision_check(near_node, new_node):
                            
                            old_parent_idx = parent[near_idx]
                            old_edge = (nodes[old_parent_idx], near_node)
                            
                            if old_edge in self.tree_edges:
                                self.tree_edges.remove(old_edge)
                            elif (near_node, nodes[old_parent_idx]) in self.tree_edges:
                                self.tree_edges.remove((near_node, nodes[old_parent_idx]))
                            
                            parent[near_idx] = new_idx
                            cost[near_idx] = new_cost
                            self.tree_edges.append((new_node, near_node))

                # Check if goal is reached
                dist_to_goal = np.linalg.norm(np.array(new_node) - np.array(self.goal))
                if dist_to_goal <= self.step_size:
                    if self.line_collision_check(new_node, self.goal):
                        final_cost = cost[new_idx] + dist_to_goal
                        
                        if goal_idx is None or final_cost < cost[goal_idx]:
                            
                            if goal_idx is None:
                                nodes.append(self.goal)
                                goal_idx = len(nodes) - 1
                            else:
                                old_parent_of_goal = nodes[parent[goal_idx]]
                                old_goal_edge = (old_parent_of_goal, self.goal)
                                if old_goal_edge in self.tree_edges:
                                    self.tree_edges.remove(old_goal_edge)
                            
                            parent[goal_idx] = new_idx
                            cost[goal_idx] = final_cost
                            self.tree_edges.append((new_node, self.goal))

            # Store snapshots for visualization
            self.history.append(list(self.tree_edges))
            current_c = cost[goal_idx] if goal_idx is not None else float('inf')
            self.cost_history.append(current_c)
            
            if goal_idx is not None:
                current_path = self.reconstruct_path(nodes, parent, goal_idx)
                self.path_history.append(current_path)
            else:
                self.path_history.append(None)

            if self.visualize_live and iteration % 20 == 0:
                self.plot_search_iteration(iteration, self.cost_history[-1], nodes, parent, goal_idx)

        # Save the final path
        if goal_idx is not None:
            final_path = self.reconstruct_path(nodes, parent, goal_idx)
            sparse_path = self.make_sparse_path(final_path, n=50)
            self.save_path(sparse_path)
            print(f"Sparse path saved to {self.path_filename}")

        if self.visualize_live:
            self.plot_search_iteration(self.max_iterations, cost[goal_idx] if goal_idx is not None else float('inf'), nodes, parent, goal_idx)
            plt.ioff()
            plt.show()


    def plot_search_iteration(self, iteration, c_best, nodes, parent, goal_idx):
        self.ax.clear()
        
        self.ax.imshow(
            self.grid,
            extent=[self.x_range[0], self.x_range[-1], self.y_range[0], self.y_range[-1]],
            origin="lower",
            cmap=self.cmap,
        )
        
        for start_node, end_node in self.tree_edges:
            self.ax.plot([start_node[0], end_node[0]], [start_node[1], end_node[1]], "b-", alpha=0.3, lw=0.5)

        if c_best < float('inf'):
            c_min = np.linalg.norm(np.array(self.goal) - np.array(self.start))
            center = (np.array(self.start) + np.array(self.goal)) / 2.0
            angle = np.degrees(np.arctan2(self.goal[1]-self.start[1], self.goal[0]-self.start[0]))
            
            width = c_best
            height = np.sqrt(max(0.1, c_best**2 - c_min**2))
            
            ell = Ellipse(xy=center, width=width, height=height, angle=angle,
                        edgecolor='r', fc='None', lw=1.5, ls='--', alpha=0.5)
            self.ax.add_patch(ell)

        if goal_idx is not None:
            path = self.reconstruct_path(nodes, parent, goal_idx)
            if path:
                path = np.array(path)
                self.ax.plot(path[:, 0], path[:, 1], "y-", lw=2, alpha=0.9, label="Best Path")

        self.ax.plot(self.start[0], self.start[1], "go", markersize=10)
        self.ax.plot(self.goal[0], self.goal[1], "ro", markersize=10)
        cost_text = f"{c_best:.2f}" if c_best < float('inf') else "inf"
        self.ax.set_title(f"Iteration {iteration} | Cost: {cost_text}")
        self.ax.legend()
        plt.pause(0.01)

    def reconstruct_path(self, nodes, parent, idx):
        path = []
        while idx is not None:
            path.append(nodes[idx])
            idx = parent[idx]
        return path[::-1]

    def make_sparse_path(self, path, n=20):
        idxs = np.linspace(0, len(path) - 1, min(n, len(path)), dtype=int)
        return [path[i] for i in idxs]

    def save_path(self, path):
        Path(self.path_filename).parent.mkdir(parents=True, exist_ok=True)
        with open(self.path_filename, "w") as f:
            json.dump(path, f)

    def visualize_search(self, gif_name, frame_skip=3):
        if not self.tree_edges:
            return

        Path(gif_name).parent.mkdir(parents=True, exist_ok=True)

        fig, ax = plt.subplots(figsize=(10, 8))
        cmap = ListedColormap([[1, 1, 1], [0, 0, 0]])

        # Only save every frame_skip frames for faster generation
        frames = list(range(0, len(self.tree_edges), frame_skip))
        print(f"Creating GIF with {len(frames)} frames (skipping every {frame_skip})...")

        def update(frame_idx):
            i = frames[frame_idx]
            ax.clear()
            
            ax.imshow(
                self.grid,
                extent=[self.x_range[0], self.x_range[-1], self.y_range[0], self.y_range[-1]],
                origin="lower",
                cmap=cmap,
            )
            
            for e in self.history[i]:
                ax.plot([e[0][0], e[1][0]], [e[0][1], e[1][1]], "b-", alpha=0.3, lw=0.5)

            c_best = self.cost_history[i]
            if c_best < float('inf'):
                c_min = np.linalg.norm(np.array(self.goal) - np.array(self.start))
                center = (np.array(self.start) + np.array(self.goal)) / 2.0
                angle = np.degrees(np.arctan2(self.goal[1]-self.start[1], self.goal[0]-self.start[0]))
                
                width = c_best
                height = np.sqrt(max(0.1, c_best**2 - c_min**2))
                
                ell = Ellipse(xy=center, width=width, height=height, angle=angle,
                            edgecolor='r', fc='None', lw=1.5, ls='--', alpha=0.5)
                ax.add_patch(ell)

            if self.path_history[i] is not None:
                path = np.array(self.path_history[i])
                ax.plot(path[:, 0], path[:, 1], "y-", lw=2, alpha=0.9, label="Best Path")

            ax.plot(self.start[0], self.start[1], "go", markersize=10)
            ax.plot(self.goal[0], self.goal[1], "ro", markersize=10)
            ax.set_title(f"Iteration {i} | Cost: {c_best:.2f}")
            
        ani = anm.FuncAnimation(fig, update, frames=len(frames), interval=40)
        ani.save(gif_name, writer=PillowWriter(fps=25))
        print(f"GIF saved to {gif_name}")
        plt.close(fig)

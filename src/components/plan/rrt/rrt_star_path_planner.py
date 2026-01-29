"""
rrt_star_path_planner.py

Author: Yuvraj Gupta
https://www.linkedin.com/in/yuvraj-gupta11/

RRT* (Rapidly-exploring Random Tree Star) path planner.
"""

import matplotlib
matplotlib.use("Agg")

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

from binary_occupancy_grid import BinaryOccupancyGrid
from min_max import MinMax


class RrtStarPathPlanner:
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
    ):
        np.random.seed(42)

        self.start = start
        self.goal = goal
        self.path_filename = path_filename
        self.max_iterations = max_iterations
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate

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
        self.visualize_search(gif_name)

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
        return 5.0 * np.sqrt(np.log(n) / n)

    def extend(self, nearest, rnd):
        d = np.array(rnd) - np.array(nearest)
        dist = np.linalg.norm(d)
        if dist < 1e-6:
            return None
        return tuple(np.array(nearest) + d / dist * min(self.step_size, dist))

    def search(self):
        nodes = [self.start]
        parent = {0: None}
        cost = {0: 0.0}

        for _ in range(self.max_iterations):
            rnd = self.goal if np.random.rand() < self.goal_sample_rate else (
                np.random.uniform(self.x_min, self.x_max),
                np.random.uniform(self.y_min, self.y_max),
            )

            nearest = self.get_nearest_node(nodes, rnd)
            nearest_idx = nodes.index(nearest)

            new_node = self.extend(nearest, rnd)
            if new_node is None or not self.line_collision_check(nearest, new_node):
                continue

            node_idx = len(nodes)
            nodes.append(new_node)
            self.explored_nodes.append(new_node)
            cost[node_idx] = float("inf")

            radius = self.get_rewire_radius(len(nodes))
            near_idxs = self.get_near_nodes(nodes, new_node, radius)

            best_parent = nearest_idx
            min_cost = cost[nearest_idx] + np.linalg.norm(
                np.array(nearest) - np.array(new_node)
            )

            for i in near_idxs:
                if self.line_collision_check(nodes[i], new_node):
                    c = cost[i] + np.linalg.norm(
                        np.array(nodes[i]) - np.array(new_node)
                    )
                    if c < min_cost:
                        min_cost = c
                        best_parent = i

            parent[node_idx] = best_parent
            cost[node_idx] = min_cost
            self.tree_edges.append((nodes[best_parent], new_node))

            for i in near_idxs:
                if i != best_parent and self.line_collision_check(new_node, nodes[i]):
                    new_cost = cost[node_idx] + np.linalg.norm(
                        np.array(nodes[i]) - np.array(new_node)
                    )
                    if new_cost < cost[i]:
                        parent[i] = node_idx
                        cost[i] = new_cost

            if np.linalg.norm(np.array(new_node) - np.array(self.goal)) < self.step_size * 2:
                if self.line_collision_check(new_node, self.goal):
                    goal_idx = len(nodes)
                    nodes.append(self.goal)
                    parent[goal_idx] = node_idx
                    self.path = self.reconstruct_path(nodes, parent, goal_idx)
                    self.save_path(self.make_sparse_path(self.path))
                    return

        closest = min(
            range(len(nodes)),
            key=lambda i: np.linalg.norm(np.array(nodes[i]) - np.array(self.goal)),
        )
        self.path = self.reconstruct_path(nodes, parent, closest)
        self.save_path(self.make_sparse_path(self.path))

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

    def visualize_search(self, gif_name):
        if not self.tree_edges:
            return

        Path(gif_name).parent.mkdir(parents=True, exist_ok=True)

        fig, ax = plt.subplots(figsize=(10, 8))
        cmap = ListedColormap([[1, 1, 1], [0, 0, 0]])

        def update(i):
            ax.clear()
            ax.imshow(
                self.grid,
                extent=[self.x_range[0], self.x_range[-1],
                        self.y_range[0], self.y_range[-1]],
                origin="lower",
                cmap=cmap,
            )
            for e in self.tree_edges[:i]:
                ax.plot([e[0][0], e[1][0]], [e[0][1], e[1][1]], "b-", alpha=0.4)
            ax.plot(self.start[0], self.start[1], "go")
            ax.plot(self.goal[0], self.goal[1], "ro")

        ani = anm.FuncAnimation(fig, update, frames=len(self.tree_edges), interval=40)
        ani.save(gif_name, writer=PillowWriter(fps=25))
        plt.close(fig)

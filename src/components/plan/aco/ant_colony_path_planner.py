"""
ant_colony_path_planner.py

Ant Colony Optimization (ACO) path planner.

The planner runs a colony of ants over a 2-D occupancy grid. Each ant
constructs a grid path from start to goal using pheromone strength and a
goal-distance heuristic. Short successful paths deposit more pheromone,
while evaporation keeps the search from locking onto early poor routes.

Constructor parameters follow the project convention used by
``AStarPathPlanner``, ``PrmPathPlanner``, ``PsoPathPlanner``, etc.

Author: Banaan Kiamanesh
GitHub: https://github.com/BanaanKiamanesh
"""

import json
import sys
from pathlib import Path

import matplotlib.animation as anm
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"

sys.path.append(abs_dir_path + relative_path + "visualization")

from min_max import MinMax  # noqa: E402


class AntColonyPathPlanner:
    """Ant Colony Optimization path planner on a 2-D occupancy grid."""

    _MOVES_4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    _MOVES_8 = [
        (-1, 0), (1, 0), (0, -1), (0, 1),
        (-1, -1), (-1, 1), (1, -1), (1, 1),
    ]

    def __init__(
        self,
        start,
        goal,
        map_file,
        *,
        x_lim=None,
        y_lim=None,
        path_filename=None,
        gif_name=None,
        n_ants=12,
        n_iterations=25,
        alpha=1.0,
        beta=5.0,
        evaporation=0.30,
        q=150.0,
        max_steps=None,
        diagonal_motion=True,
        elite_fraction=0.10,
        stagnation_limit=8,
        seed=10,
    ):
        self.start = tuple(start)
        self.goal = tuple(goal)
        self.n_ants = n_ants
        self.n_iterations = n_iterations
        self.alpha = alpha
        self.beta = beta
        self.evaporation = evaporation
        self.q = q
        self.diagonal_motion = diagonal_motion
        self.elite_fraction = elite_fraction
        self.stagnation_limit = stagnation_limit
        self._rng = np.random.default_rng(seed)

        self.grid = self._load_grid(map_file)
        if x_lim is None or y_lim is None:
            raise ValueError("x_lim and y_lim are required for world/grid conversion.")

        x_min, x_max = x_lim.min_value(), x_lim.max_value()
        y_min, y_max = y_lim.min_value(), y_lim.max_value()
        self.resolution = (x_max - x_min) / self.grid.shape[1]
        self.x_range = np.arange(x_min, x_max, self.resolution)
        self.y_range = np.arange(y_min, y_max, self.resolution)
        self.rows, self.cols = self.grid.shape
        self.max_steps = max_steps or 3 * (self.rows + self.cols)

        # BinaryOccupancyGrid stores free cells as 0.0, clearance as 0.75,
        # and obstacles as 1.0. Planners in this repo avoid clearance cells.
        self.free_mask = np.isclose(self.grid, 0.0)
        self.start_idx = self._world_to_grid(self.start)
        self.goal_idx = self._world_to_grid(self.goal)
        self._validate_endpoint(self.start_idx, "start")
        self._validate_endpoint(self.goal_idx, "goal")

        self.pheromone = np.ones((self.rows, self.cols), dtype=float)
        self.pheromone[~self.free_mask] = 0.0
        self.heuristic = self._build_heuristic()
        self.heuristic_weight = self.heuristic ** self.beta
        self.neighbor_map = self._build_neighbor_map()

        self.best_path = None
        self.best_cost = np.inf
        self.best_cost_history = []
        self.iteration_best_paths = []
        self.path = []
        self.grid_path = []

        self.search()

        if path_filename and self.path:
            sparse = self._make_sparse_path(self.path)
            self._save_path(sparse, path_filename)

        self.visualize_search(gif_name)

    @staticmethod
    def _load_grid(file_path):
        ext = Path(file_path).suffix
        if ext == ".npy":
            return np.load(file_path)
        if ext == ".png":
            grid = plt.imread(file_path)
            if grid.ndim == 3:
                grid = np.mean(grid, axis=2)
            return (grid > 0.5).astype(float)
        if ext == ".json":
            with open(file_path, "r") as f:
                return np.array(json.load(f))
        raise ValueError(f"Unsupported grid format: {ext}")

    def _world_to_grid(self, point):
        gx = int((point[0] - self.x_range[0]) / self.resolution)
        gy = int((point[1] - self.y_range[0]) / self.resolution)
        return gx, gy

    def _grid_to_world(self, node):
        return (
            self.x_range[0] + node[0] * self.resolution,
            self.y_range[0] + node[1] * self.resolution,
        )

    def _in_bounds(self, node):
        return 0 <= node[0] < self.cols and 0 <= node[1] < self.rows

    def _is_free_idx(self, node):
        return self._in_bounds(node) and self.free_mask[node[1], node[0]]

    def _validate_endpoint(self, node, name):
        if not self._in_bounds(node):
            raise ValueError(f"{name.capitalize()} point is outside the map.")
        if not self._is_free_idx(node):
            raise ValueError(f"{name.capitalize()} point is inside an obstacle.")

    def _build_heuristic(self):
        yy, xx = np.indices((self.rows, self.cols))
        gx, gy = self.goal_idx
        dist = np.hypot(xx - gx, yy - gy)
        heuristic = 1.0 / (dist + 1e-6)
        heuristic[~self.free_mask] = 0.0
        return heuristic

    def _build_neighbor_map(self):
        neighbor_map = [[tuple() for _ in range(self.cols)] for _ in range(self.rows)]
        for y in range(self.rows):
            for x in range(self.cols):
                node = (x, y)
                if self._is_free_idx(node):
                    neighbor_map[y][x] = tuple(self._neighbors_for_node(node))
        return neighbor_map

    def _neighbors_for_node(self, node):
        moves = self._MOVES_8 if self.diagonal_motion else self._MOVES_4
        x, y = node
        result = []

        for dx, dy in moves:
            next_node = (x + dx, y + dy)
            if not self._is_free_idx(next_node):
                continue

            if abs(dx) == 1 and abs(dy) == 1:
                if not self._is_free_idx((x + dx, y)):
                    continue
                if not self._is_free_idx((x, y + dy)):
                    continue

            result.append(next_node)

        return result

    def _neighbors(self, node):
        return self.neighbor_map[node[1]][node[0]]

    @staticmethod
    def _distance(a, b):
        return float(np.hypot(a[0] - b[0], a[1] - b[1]))

    def _path_cost(self, path):
        cost = 0.0
        for i in range(len(path) - 1):
            cost += self._distance(path[i], path[i + 1]) * self.resolution
        return cost

    def _choose_next(self, candidates, visited):
        weights = []
        total = 0.0

        for node in candidates:
            x, y = node
            tau = self.pheromone[y, x] ** self.alpha
            revisit_penalty = 0.25 if node in visited else 1.0
            weight = tau * self.heuristic_weight[y, x] * revisit_penalty
            weights.append(weight)
            total += weight

        if total <= 0.0 or not np.isfinite(total):
            return candidates[int(self._rng.integers(len(candidates)))]

        threshold = self._rng.random() * total
        cumulative = 0.0
        for node, weight in zip(candidates, weights):
            cumulative += weight
            if cumulative >= threshold:
                return node
        return candidates[-1]

    def _construct_path(self):
        current = self.start_idx
        path = [current]
        visited = {current}

        for _ in range(self.max_steps):
            if current == self.goal_idx:
                return self._remove_cycles(path)

            candidates = self._neighbors(current)
            if not candidates:
                return None

            if self.goal_idx in candidates:
                path.append(self.goal_idx)
                return self._remove_cycles(path)

            unvisited = [node for node in candidates if node not in visited]
            usable_candidates = unvisited if unvisited else candidates
            current = self._choose_next(usable_candidates, visited)
            path.append(current)
            visited.add(current)

        if path[-1] == self.goal_idx:
            return self._remove_cycles(path)
        return None

    @staticmethod
    def _remove_cycles(path):
        cleaned = []
        index_by_node = {}
        for node in path:
            if node in index_by_node:
                keep_until = index_by_node[node] + 1
                for removed in cleaned[keep_until:]:
                    index_by_node.pop(removed, None)
                cleaned = cleaned[:keep_until]
                continue
            index_by_node[node] = len(cleaned)
            cleaned.append(node)
        return cleaned

    def _evaporate_pheromone(self):
        self.pheromone *= 1.0 - self.evaporation
        self.pheromone[~self.free_mask] = 0.0
        self.pheromone = np.maximum(self.pheromone, 1e-6)
        self.pheromone[~self.free_mask] = 0.0

    def _deposit_pheromone(self, path, cost, multiplier=1.0):
        if cost <= 0.0:
            return
        amount = multiplier * self.q / cost
        for x, y in path:
            self.pheromone[y, x] += amount

    def search(self):
        """Run the ant colony search and populate ``path`` if successful."""
        stale_iterations = 0

        for _ in range(self.n_iterations):
            iteration_paths = []
            improved = False

            for _ in range(self.n_ants):
                path = self._construct_path()
                if path is None:
                    continue

                cost = self._path_cost(path)
                iteration_paths.append((path, cost))

                if cost < self.best_cost:
                    self.best_cost = cost
                    self.best_path = path
                    improved = True

            self._evaporate_pheromone()

            if iteration_paths:
                iteration_paths.sort(key=lambda item: item[1])
                n_elite = max(1, int(len(iteration_paths) * self.elite_fraction))

                for path, cost in iteration_paths[:n_elite]:
                    self._deposit_pheromone(path, cost, multiplier=1.0)

                if self.best_path is not None:
                    self._deposit_pheromone(
                        self.best_path,
                        self.best_cost,
                        multiplier=2.0,
                    )

            self.best_cost_history.append(self.best_cost)
            self.iteration_best_paths.append(
                list(self.best_path) if self.best_path is not None else None
            )

            if improved:
                stale_iterations = 0
            elif self.best_path is not None:
                stale_iterations += 1

            if (
                self.best_path is not None
                and self.stagnation_limit is not None
                and stale_iterations >= self.stagnation_limit
            ):
                break

        if self.best_path is None:
            print("ACO: no feasible path found.")
            return

        self.grid_path = self._densify_path(
            self._shortcut_path(self._prune_collinear(self.best_path))
        )
        self.path = [self._grid_to_world(node) for node in self.grid_path]
        print(
            f"ACO: best path cost={self.best_cost:.2f}, "
            f"grid_waypoints={len(self.grid_path)}"
        )

    def run(self):
        """Return the best grid path and its world-coordinate cost."""
        return self.best_path, self.best_cost

    @staticmethod
    def _prune_collinear(path):
        if len(path) <= 2:
            return path

        pruned = [path[0]]
        last_dir = (
            np.sign(path[1][0] - path[0][0]),
            np.sign(path[1][1] - path[0][1]),
        )

        for i in range(1, len(path) - 1):
            current_dir = (
                np.sign(path[i + 1][0] - path[i][0]),
                np.sign(path[i + 1][1] - path[i][1]),
            )
            if current_dir != last_dir:
                pruned.append(path[i])
                last_dir = current_dir

        pruned.append(path[-1])
        return pruned

    def _line_is_free(self, start, goal):
        dx = goal[0] - start[0]
        dy = goal[1] - start[1]
        steps = max(abs(dx), abs(dy)) * 2
        if steps == 0:
            return self._is_free_idx(start)

        for i in range(steps + 1):
            t = i / steps
            node = (
                int(round(start[0] + dx * t)),
                int(round(start[1] + dy * t)),
            )
            if not self._is_free_idx(node):
                return False
        return True

    def _shortcut_path(self, path):
        if len(path) <= 2:
            return path

        shortened = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and not self._line_is_free(path[i], path[j]):
                j -= 1
            shortened.append(path[j])
            i = j
        return shortened

    @staticmethod
    def _densify_path(path, max_step_cells=8):
        if len(path) <= 1:
            return path

        dense = [path[0]]
        for start, goal in zip(path[:-1], path[1:]):
            dx = goal[0] - start[0]
            dy = goal[1] - start[1]
            steps = max(1, int(np.ceil(max(abs(dx), abs(dy)) / max_step_cells)))
            for step in range(1, steps + 1):
                node = (
                    int(round(start[0] + dx * step / steps)),
                    int(round(start[1] + dy * step / steps)),
                )
                if node != dense[-1]:
                    dense.append(node)
        return dense

    def _make_sparse_path(self, path, num_points=20):
        if len(path) <= num_points:
            return [list(p) for p in path]
        indices = np.linspace(0, len(path) - 1, num_points, dtype=int)
        return [list(path[i]) for i in indices]

    @staticmethod
    def _save_path(path, filename):
        Path(filename).parent.mkdir(parents=True, exist_ok=True)
        with open(filename, "w") as f:
            json.dump(path, f)

    def visualize_search(self, gif_name=None):
        """Render a GIF showing the best ACO path improving by iteration."""
        if gif_name is None:
            return
        if not self.iteration_best_paths:
            return

        cmap = ListedColormap([
            [1.0, 1.0, 1.0],
            [0.5, 0.5, 0.5],
            [0.0, 0.0, 0.0],
        ])

        def _disc(grid):
            out = np.zeros_like(grid, dtype=int)
            out[np.isclose(grid, 0.75)] = 1
            out[grid >= 0.99] = 2
            return out

        grid_disc = _disc(self.grid)
        max_search_frames = 80
        step = max(1, len(self.iteration_best_paths) // max_search_frames)
        search_frames = list(range(0, len(self.iteration_best_paths), step))
        if search_frames[-1] != len(self.iteration_best_paths) - 1:
            search_frames.append(len(self.iteration_best_paths) - 1)

        path_frames = 20
        hold_frames = 15
        offsets = np.cumsum([0, len(search_frames), path_frames, hold_frames])
        total = int(offsets[-1])

        def _phase(i):
            for phase in range(3):
                if i < offsets[phase + 1]:
                    return phase, i - int(offsets[phase])
            return 2, hold_frames - 1

        def _plot_grid_path(ax, grid_path, **kwargs):
            if not grid_path or len(grid_path) < 2:
                return
            points = [self._grid_to_world(node) for node in grid_path]
            ax.plot(
                [p[0] for p in points],
                [p[1] for p in points],
                **kwargs,
            )

        def update(i, ax):
            phase, local = _phase(i)
            ax.clear()
            ax.imshow(
                grid_disc,
                extent=[
                    self.x_range[0],
                    self.x_range[-1],
                    self.y_range[0],
                    self.y_range[-1],
                ],
                origin="lower",
                cmap=cmap,
                vmin=0,
                vmax=2,
                alpha=0.85,
            )

            if phase == 0:
                idx = search_frames[min(local, len(search_frames) - 1)]
                candidate = self.iteration_best_paths[idx]
                _plot_grid_path(
                    ax,
                    candidate,
                    color="#1565C0",
                    linewidth=2.0,
                    alpha=0.85,
                    label="Best Path",
                )
                cost = self.best_cost_history[idx]
                cost_text = f"{cost:.1f}" if np.isfinite(cost) else "pending"
                ax.set_title(
                    f"ACO Search ({idx + 1}/{len(self.iteration_best_paths)}) | cost={cost_text}",
                    fontsize=14,
                )
            else:
                if self.grid_path:
                    if phase == 1:
                        n = max(
                            1,
                            int(len(self.grid_path) * (local + 1) / path_frames),
                        )
                    else:
                        n = len(self.grid_path)
                    _plot_grid_path(
                        ax,
                        self.grid_path[:n],
                        color="#2E7D32",
                        linewidth=3.0,
                        label="Path",
                    )
                ax.set_title("ACO Final Path", fontsize=14)

            ax.plot(self.start[0], self.start[1], "go", markersize=10, label="Start")
            ax.plot(self.goal[0], self.goal[1], "ro", markersize=10, label="Goal")
            ax.legend(loc="upper left")
            ax.set_xlabel("X [m]", fontsize=12)
            ax.set_ylabel("Y [m]", fontsize=12)
            ax.set_aspect("equal")

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111)
        ax.set_aspect("equal")

        print(f"ACO search animation: {total} frames")
        anime = anm.FuncAnimation(
            fig,
            update,
            fargs=(ax,),
            frames=total,
            interval=60,
            repeat=False,
        )
        try:
            anime.save(gif_name, writer="pillow", fps=12)
            print(f"Search GIF saved to {gif_name}")
        except Exception as e:
            print(f"Error saving search GIF: {e}")
        plt.clf()
        plt.close()


if __name__ == "__main__":
    map_file = "map.json"
    path_file = "path.json"
    gif_path = "aco_search.gif"

    x_lim, y_lim = MinMax(-5, 55), MinMax(-20, 25)
    start = (0, 0)
    goal = (50, -10)

    planner = AntColonyPathPlanner(
        start,
        goal,
        map_file,
        x_lim=x_lim,
        y_lim=y_lim,
        path_filename=path_file,
        gif_name=gif_path,
    )

"""
genetic_algorithm_path_planner.py

Genetic Algorithm (GA) path planner with spline smoothing.

The planner optimizes intermediate world-coordinate control points. Each
chromosome is converted to a Catmull-Rom spline and scored by path length,
map collisions, map-bound violations, and curvature. The initial
population is seeded with a feasible grid route so the sample remains
deterministic on the project path-planning scenario.

Constructor parameters follow the project convention used by
``AStarPathPlanner``, ``PrmPathPlanner``, ``PsoPathPlanner``, etc.

Author: Banaan Kiamanesh
GitHub: https://github.com/BanaanKiamanesh
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
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


@dataclass(frozen=True)
class GAConfig:
    num_control_points: int = 7
    population_size: int = 60
    generations: int = 90
    tournament_size: int = 3
    elite_count: int = 4
    crossover_rate: float = 0.85
    mutation_rate: float = 0.16
    mutation_sigma: float = 2.0
    samples_per_segment: int = 10
    seed: int = 42


@dataclass
class PlannerResult:
    control_points: np.ndarray
    spline_points: np.ndarray
    cost_history: list[float]
    best_cost: float
    path_length: float
    collision_sample_count: int
    bounds_sample_count: int


def catmull_rom_spline(points: np.ndarray, samples_per_segment: int) -> np.ndarray:
    points = np.asarray(points, dtype=float)

    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError("points must have shape (n, 2).")

    if len(points) < 2:
        raise ValueError("At least start and goal points are required.")

    padded = np.vstack([points[0], points, points[-1]])
    t = np.linspace(0.0, 1.0, samples_per_segment, endpoint=False)
    t2 = t * t
    t3 = t2 * t

    segments = []
    for i in range(1, len(padded) - 2):
        p0 = padded[i - 1]
        p1 = padded[i]
        p2 = padded[i + 1]
        p3 = padded[i + 2]

        segment = 0.5 * (
            (2.0 * p1)
            + np.outer(t, -p0 + p2)
            + np.outer(t2, 2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3)
            + np.outer(t3, -p0 + 3.0 * p1 - 3.0 * p2 + p3)
        )
        segments.append(segment)

    return np.vstack(segments + [points[-1][None, :]])


def path_length(path: np.ndarray) -> float:
    return float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))


def curvature_cost(path: np.ndarray) -> float:
    if len(path) < 3:
        return 0.0

    v1 = np.diff(path[:-1], axis=0)
    v2 = np.diff(path[1:], axis=0)
    n1 = np.linalg.norm(v1, axis=1) + 1e-9
    n2 = np.linalg.norm(v2, axis=1) + 1e-9
    cos_angles = np.sum(v1 * v2, axis=1) / (n1 * n2)
    angles = np.arccos(np.clip(cos_angles, -1.0, 1.0))
    return float(np.sum(angles * angles))


class GeneticAlgorithmPathPlanner:
    """Genetic Algorithm spline planner on a 2-D occupancy grid."""

    _MOVES_8 = [
        (-1, 0),
        (1, 0),
        (0, -1),
        (0, 1),
        (-1, -1),
        (-1, 1),
        (1, -1),
        (1, 1),
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
        config=None,
        num_control_points=7,
        population_size=60,
        generations=90,
        seed=42,
    ):
        self.start = np.array(start, dtype=float)
        self.goal = np.array(goal, dtype=float)
        self.config = config or GAConfig(
            num_control_points=num_control_points,
            population_size=population_size,
            generations=generations,
            seed=seed,
        )
        self.rng = np.random.default_rng(self.config.seed)

        if self.config.elite_count >= self.config.population_size:
            raise ValueError("elite_count must be smaller than population_size.")
        if self.config.num_control_points < 1:
            raise ValueError("num_control_points must be at least 1.")

        self.raw_grid = self._load_grid(map_file)
        if x_lim is None or y_lim is None:
            raise ValueError("x_lim and y_lim are required for world/grid conversion.")

        self.x_min, self.x_max = x_lim.min_value(), x_lim.max_value()
        self.y_min, self.y_max = y_lim.min_value(), y_lim.max_value()
        self.bounds = (self.x_min, self.x_max, self.y_min, self.y_max)
        self.resolution = (self.x_max - self.x_min) / self.raw_grid.shape[1]
        self.x_range = np.arange(self.x_min, self.x_max, self.resolution)
        self.y_range = np.arange(self.y_min, self.y_max, self.resolution)

        # BinaryOccupancyGrid stores free cells as 0.0, clearance as 0.75,
        # and obstacles as 1.0. This planner treats every nonzero cell as
        # impassable, matching the other path-planning samples.
        self.grid = (self.raw_grid > 0).astype(np.uint8)
        self.rows, self.cols = self.grid.shape

        self.start_idx = self._world_to_grid_rc(self.start)
        self.goal_idx = self._world_to_grid_rc(self.goal)
        self._validate_endpoint(self.start_idx, "start")
        self._validate_endpoint(self.goal_idx, "goal")

        self.seed_grid_path = self._shortest_path_bfs()
        if not self.seed_grid_path:
            raise ValueError("No feasible path exists between start and goal.")
        self.seed_path = np.array(
            [self._grid_to_world(node) for node in self.seed_grid_path],
            dtype=float,
        )

        self.cost_history = []
        self.best_path_history = []
        self.result = self.plan()
        self.path = [tuple(point) for point in self.result.spline_points]
        self.path = self._make_sparse_path(self.path)

        if path_filename and self.path:
            self._save_path(self.path, path_filename)

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

    def _world_to_grid_rc(self, point):
        col = int((point[0] - self.x_min) / self.resolution)
        row = int((point[1] - self.y_min) / self.resolution)
        return row, col

    def _grid_to_world(self, node):
        row, col = node
        return (
            self.x_min + col * self.resolution,
            self.y_min + row * self.resolution,
        )

    def _in_bounds_idx(self, node):
        return 0 <= node[0] < self.rows and 0 <= node[1] < self.cols

    def _is_free_idx(self, node):
        return self._in_bounds_idx(node) and self.grid[node] == 0

    def _validate_endpoint(self, node, name):
        if not self._in_bounds_idx(node):
            raise ValueError(f"{name.capitalize()} point is outside the map.")
        if not self._is_free_idx(node):
            raise ValueError(f"{name.capitalize()} point is inside an obstacle.")

    def _shortest_path_bfs(self):
        queue = deque([self.start_idx])
        previous = {self.start_idx: None}

        while queue:
            current = queue.popleft()
            if current == self.goal_idx:
                break

            for dr, dc in self._MOVES_8:
                next_node = (current[0] + dr, current[1] + dc)
                if not self._is_free_idx(next_node):
                    continue
                if abs(dr) == 1 and abs(dc) == 1:
                    if not self._is_free_idx((current[0] + dr, current[1])):
                        continue
                    if not self._is_free_idx((current[0], current[1] + dc)):
                        continue
                if next_node not in previous:
                    previous[next_node] = current
                    queue.append(next_node)

        if self.goal_idx not in previous:
            return []

        path = []
        current = self.goal_idx
        while current is not None:
            path.append(current)
            current = previous[current]
        return path[::-1]

    def plan(self, verbose=False):
        population = self._initial_population()
        best_cost = np.inf
        best_chromosome = None
        best_path = None

        for generation in range(self.config.generations):
            scores, paths = self._evaluate_population(population)
            order = np.argsort(scores)

            if scores[order[0]] < best_cost:
                best_cost = float(scores[order[0]])
                best_chromosome = population[order[0]].copy()
                best_path = paths[order[0]]

            self.cost_history.append(best_cost)
            self.best_path_history.append(best_path.copy() if best_path is not None else None)

            if verbose and (generation % 20 == 0 or generation == self.config.generations - 1):
                print(f"generation={generation:03d} best_cost={best_cost:.4f}")

            population = self._next_generation(population, scores, order)

        scores, paths = self._evaluate_population(population)
        index = int(np.argmin(scores))
        if scores[index] < best_cost:
            best_cost = float(scores[index])
            best_chromosome = population[index].copy()
            best_path = paths[index]

        if best_chromosome is None or best_path is None:
            raise RuntimeError("The planner did not produce a valid candidate.")

        _, collision_count = self._collision_violation(best_path)
        _, bounds_count = self._bounds_violation(best_path)

        if collision_count > 0 or bounds_count > 0:
            best_path = self._fallback_seed_path()
            best_chromosome = self._control_points_from_path(best_path).ravel()
            best_cost = self._evaluate_chromosome(best_chromosome)[0]
            _, collision_count = self._collision_violation(best_path)
            _, bounds_count = self._bounds_violation(best_path)

        print(
            f"GA: best path cost={best_cost:.2f}, "
            f"collisions={collision_count}, bounds={bounds_count}"
        )

        return PlannerResult(
            control_points=best_chromosome.reshape(self.config.num_control_points, 2),
            spline_points=best_path,
            cost_history=list(self.cost_history),
            best_cost=best_cost,
            path_length=path_length(best_path),
            collision_sample_count=collision_count,
            bounds_sample_count=bounds_count,
        )

    def _initial_population(self):
        cfg = self.config
        dim = cfg.num_control_points * 2
        population = np.empty((cfg.population_size, dim), dtype=float)
        seed_control = self._control_points_from_path(self.seed_path)
        corridor_offsets = self._corridor_offsets()

        for i in range(cfg.population_size):
            if i == 0:
                control_points = seed_control.copy()
            elif i < cfg.population_size // 2:
                control_points = seed_control + self.rng.normal(
                    0.0,
                    1.0,
                    size=seed_control.shape,
                )
            else:
                offset = float(self.rng.choice(corridor_offsets))
                control_points = self._corridor_control_points(
                    offset,
                    noise_sigma=1.0,
                )

            population[i] = self._clip_control_points(control_points).ravel()

        return population

    def _control_points_from_path(self, path):
        path = np.asarray(path, dtype=float)
        if len(path) < 2:
            raise ValueError("A path must contain at least start and goal.")

        distances = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        total = distances[-1]
        if total <= 1e-9:
            return np.repeat(self.start[None, :], self.config.num_control_points, axis=0)

        targets = np.linspace(0.0, total, self.config.num_control_points + 2)[1:-1]
        control_points = []
        for target in targets:
            index = int(np.searchsorted(distances, target))
            index = min(max(index, 1), len(path) - 1)
            left = path[index - 1]
            right = path[index]
            span = distances[index] - distances[index - 1]
            ratio = 0.0 if span <= 1e-9 else (target - distances[index - 1]) / span
            control_points.append(left + ratio * (right - left))

        return np.array(control_points, dtype=float)

    def _corridor_offsets(self):
        average_span = 0.5 * ((self.x_max - self.x_min) + (self.y_max - self.y_min))
        return [
            0.0,
            0.12 * average_span,
            -0.12 * average_span,
            0.24 * average_span,
            -0.24 * average_span,
            0.36 * average_span,
            -0.36 * average_span,
        ]

    def _corridor_control_points(self, offset, noise_sigma):
        cfg = self.config
        direction = self.goal - self.start
        norm = float(np.linalg.norm(direction))

        if norm < 1e-12:
            perpendicular = np.array([0.0, 1.0])
        else:
            perpendicular = np.array([-direction[1], direction[0]]) / norm

        control_points = []
        for j in range(1, cfg.num_control_points + 1):
            t = j / (cfg.num_control_points + 1)
            base = (1.0 - t) * self.start + t * self.goal
            point = base + offset * np.sin(np.pi * t) * perpendicular
            point += self.rng.normal(0.0, noise_sigma, size=2)
            control_points.append(point)

        return np.array(control_points, dtype=float)

    def _clip_control_points(self, control_points):
        clipped = np.array(control_points, dtype=float, copy=True)
        clipped[:, 0] = np.clip(clipped[:, 0], self.x_min, self.x_max - self.resolution)
        clipped[:, 1] = np.clip(clipped[:, 1], self.y_min, self.y_max - self.resolution)
        return clipped

    def _evaluate_population(self, population):
        scores = np.empty(len(population), dtype=float)
        paths = []

        for i, chromosome in enumerate(population):
            scores[i], candidate_path = self._evaluate_chromosome(chromosome)
            paths.append(candidate_path)

        return scores, paths

    def _evaluate_chromosome(self, chromosome):
        cfg = self.config
        control_points = chromosome.reshape(cfg.num_control_points, 2)
        base_points = np.vstack([self.start, control_points, self.goal])
        spline_path = catmull_rom_spline(base_points, cfg.samples_per_segment)

        length_term = path_length(spline_path)
        collision_sum, collision_count = self._collision_violation(spline_path)
        bounds_sum, bounds_count = self._bounds_violation(spline_path)
        smoothness_term = curvature_cost(spline_path)

        cost = (
            length_term
            + 20000.0 * collision_sum
            + 500.0 * collision_count
            + 20000.0 * bounds_sum
            + 500.0 * bounds_count
            + 0.15 * smoothness_term
        )

        return float(cost), spline_path

    def _collision_violation(self, path):
        total_violation = 0.0
        collision_count = 0

        for point in path:
            node = self._world_to_grid_rc(point)
            if not self._in_bounds_idx(node):
                continue
            value = self.raw_grid[node]
            if value > 0:
                collision_count += 1
                total_violation += float(value * value)

        return total_violation, collision_count

    def _bounds_violation(self, path):
        xmin, xmax, ymin, ymax = self.bounds
        violations = (
            np.maximum(0.0, xmin - path[:, 0])
            + np.maximum(0.0, path[:, 0] - (xmax - self.resolution))
            + np.maximum(0.0, ymin - path[:, 1])
            + np.maximum(0.0, path[:, 1] - (ymax - self.resolution))
        )

        return float(np.sum(violations * violations)), int(np.count_nonzero(violations > 0.0))

    def _next_generation(self, population, scores, order):
        cfg = self.config
        next_population = [population[index].copy() for index in order[: cfg.elite_count]]

        while len(next_population) < cfg.population_size:
            parent_1 = self._tournament_select(population, scores)
            parent_2 = self._tournament_select(population, scores)
            child_1, child_2 = self._crossover(parent_1, parent_2)

            for child in (child_1, child_2):
                self._mutate(child)
                next_population.append(child)
                if len(next_population) >= cfg.population_size:
                    break

        return np.array(next_population, dtype=float)

    def _tournament_select(self, population, scores):
        indices = self.rng.integers(0, len(population), self.config.tournament_size)
        winner_index = indices[np.argmin(scores[indices])]
        return population[winner_index].copy()

    def _crossover(self, parent_1, parent_2):
        if self.rng.random() >= self.config.crossover_rate:
            return parent_1.copy(), parent_2.copy()

        alpha = self.rng.uniform(-0.15, 1.15, size=len(parent_1))
        child_1 = alpha * parent_1 + (1.0 - alpha) * parent_2
        child_2 = alpha * parent_2 + (1.0 - alpha) * parent_1

        return child_1, child_2

    def _mutate(self, chromosome):
        cfg = self.config
        mutation_mask = self.rng.random(len(chromosome)) < cfg.mutation_rate
        mutation_count = int(np.count_nonzero(mutation_mask))

        if mutation_count > 0:
            chromosome[mutation_mask] += self.rng.normal(
                0.0,
                cfg.mutation_sigma,
                size=mutation_count,
            )

        if self.rng.random() < 0.04:
            point_index = int(self.rng.integers(0, cfg.num_control_points))
            chromosome[2 * point_index] = self.rng.uniform(self.x_min, self.x_max)
            chromosome[2 * point_index + 1] = self.rng.uniform(self.y_min, self.y_max)

        control_points = chromosome.reshape(cfg.num_control_points, 2)
        chromosome[:] = self._clip_control_points(control_points).ravel()

    def _fallback_seed_path(self):
        seed_control = self._control_points_from_path(self.seed_path)
        base_points = np.vstack([self.start, seed_control, self.goal])
        spline_path = catmull_rom_spline(base_points, self.config.samples_per_segment)
        _, collision_count = self._collision_violation(spline_path)
        _, bounds_count = self._bounds_violation(spline_path)
        if collision_count == 0 and bounds_count == 0:
            return spline_path
        return self.seed_path

    def _make_sparse_path(self, path, num_points=20):
        if len(path) <= num_points:
            return [list(point) for point in path]
        indices = np.linspace(0, len(path) - 1, num_points, dtype=int)
        return [list(path[i]) for i in indices]

    @staticmethod
    def _save_path(path, filename):
        Path(filename).parent.mkdir(parents=True, exist_ok=True)
        with open(filename, "w") as f:
            json.dump(path, f)

    def visualize_search(self, gif_name=None):
        """Render a GIF showing GA convergence and the final spline path."""
        if gif_name is None:
            return
        if not self.best_path_history:
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

        grid_disc = _disc(self.raw_grid)
        max_search_frames = 70
        step = max(1, len(self.best_path_history) // max_search_frames)
        search_frames = list(range(0, len(self.best_path_history), step))
        if search_frames[-1] != len(self.best_path_history) - 1:
            search_frames.append(len(self.best_path_history) - 1)

        path_frames = 20
        hold_frames = 15
        offsets = np.cumsum([0, len(search_frames), path_frames, hold_frames])
        total = int(offsets[-1])

        def _phase(i):
            for phase in range(3):
                if i < offsets[phase + 1]:
                    return phase, i - int(offsets[phase])
            return 2, hold_frames - 1

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
                candidate = self.best_path_history[idx]
                if candidate is not None:
                    ax.plot(
                        candidate[:, 0],
                        candidate[:, 1],
                        color="#1565C0",
                        linewidth=2.0,
                        alpha=0.85,
                        label="Best Spline",
                    )
                ax.set_title(
                    f"GA Search ({idx + 1}/{self.config.generations}) | "
                    f"cost={self.cost_history[idx]:.1f}",
                    fontsize=14,
                )
            else:
                final_path = self.result.spline_points
                if phase == 1:
                    n = max(2, int(len(final_path) * (local + 1) / path_frames))
                else:
                    n = len(final_path)
                ax.plot(
                    final_path[:n, 0],
                    final_path[:n, 1],
                    color="#2E7D32",
                    linewidth=3.0,
                    label="Path",
                )
                base_points = np.vstack([self.start, self.result.control_points, self.goal])
                ax.plot(
                    base_points[:, 0],
                    base_points[:, 1],
                    "--o",
                    color="#607D8B",
                    linewidth=1.0,
                    markersize=3,
                    alpha=0.75,
                    label="Control Points",
                )
                ax.set_title("GA Final Spline Path", fontsize=14)

            ax.plot(self.start[0], self.start[1], "go", markersize=10, label="Start")
            ax.plot(self.goal[0], self.goal[1], "ro", markersize=10, label="Goal")
            ax.legend(loc="upper left")
            ax.set_xlabel("X [m]", fontsize=12)
            ax.set_ylabel("Y [m]", fontsize=12)
            ax.set_aspect("equal")

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111)
        ax.set_aspect("equal")

        print(f"GA search animation: {total} frames")
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
    gif_path = "ga_search.gif"

    x_lim, y_lim = MinMax(-5, 55), MinMax(-20, 25)
    start = (0, 0)
    goal = (50, -10)

    planner = GeneticAlgorithmPathPlanner(
        start,
        goal,
        map_file,
        x_lim=x_lim,
        y_lim=y_lim,
        path_filename=path_file,
        gif_name=gif_path,
    )

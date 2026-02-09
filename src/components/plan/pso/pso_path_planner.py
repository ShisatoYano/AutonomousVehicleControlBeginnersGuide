"""
pso_path_planner.py

Particle Swarm Optimization (PSO) path planner.

Each particle encodes a candidate path as a sequence of *n_waypoints*
intermediate (x, y) waypoints between the fixed start and goal.
The fitness function is the total path length with a heavy collision
penalty for any segment that intersects an obstacle or clearance zone.

Particles maintain velocities, personal bests, and track a global best.
Over many iterations the swarm converges toward a short, collision-free
path.

Reference:
    Kennedy & Eberhart,
    "Particle Swarm Optimization", IEEE ICNN, 1995.

Author: Erwin Lejeune
"""

import numpy as np
import json
import matplotlib.pyplot as plt
import matplotlib.animation as anm
import sys
from pathlib import Path
from matplotlib.colors import ListedColormap

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"

sys.path.append(abs_dir_path + relative_path + "visualization")
sys.path.append(abs_dir_path + relative_path + "state")
sys.path.append(abs_dir_path + relative_path + "obstacle")
sys.path.append(abs_dir_path + relative_path + "mapping/grid")

from min_max import MinMax  # noqa: E402


class PsoPathPlanner:
    """
    PSO path planner on a 2-D occupancy grid.

    1. Initialise a swarm of particles, each encoding *n_waypoints*
       intermediate (x, y) positions between start and goal.
    2. Evaluate fitness = path length + collision penalty.
    3. Update velocities using personal best and global best.
    4. Iterate for *max_iter* generations.
    5. Return the global best path and optionally save a search GIF.

    Constructor parameters follow the project convention used by
    ``AStarPathPlanner``, ``RrtPathPlanner``, etc.
    """

    # Collision penalty per colliding segment
    COLLISION_PENALTY = 1e4

    def __init__(self, start, goal, map_file, *,
                 x_lim=None, y_lim=None,
                 path_filename=None, gif_name=None,
                 n_particles=40, n_waypoints=5,
                 max_iter=120, w=0.6, c1=1.8, c2=1.8,
                 line_check_samples=20, seed=42):
        self.start = np.array(start, dtype=float)
        self.goal = np.array(goal, dtype=float)
        self.n_particles = n_particles
        self.n_waypoints = n_waypoints
        self.max_iter = max_iter
        self.w = w        # inertia weight
        self.c1 = c1      # cognitive coefficient
        self.c2 = c2      # social coefficient
        self.line_check_samples = line_check_samples

        # Load grid
        self.grid = self._load_grid(map_file)
        x_min, x_max = x_lim.min_value(), x_lim.max_value()
        y_min, y_max = y_lim.min_value(), y_lim.max_value()
        self.resolution = (x_max - x_min) / self.grid.shape[1]
        self.x_range = np.arange(x_min, x_max, self.resolution)
        self.y_range = np.arange(y_min, y_max, self.resolution)
        self.rows, self.cols = self.grid.shape
        self.x_min, self.x_max = x_min, x_max
        self.y_min, self.y_max = y_min, y_max

        self._rng = np.random.default_rng(seed)

        # Run PSO
        self.path = []
        self._history = []  # list of (positions_array, gbest_path) per iter
        self._run_pso()

        # Save sparse path
        if path_filename and len(self.path) > 0:
            sparse = self._make_sparse_path(self.path)
            self._save_path(sparse, path_filename)

        # Visualise
        self.visualize_search(gif_name)

    # -- Grid I/O ----------------------------------------------------------

    @staticmethod
    def _load_grid(file_path):
        ext = Path(file_path).suffix
        if ext == '.npy':
            return np.load(file_path)
        if ext == '.png':
            g = plt.imread(file_path)
            if g.ndim == 3:
                g = np.mean(g, axis=2)
            return (g > 0.5).astype(float)
        if ext == '.json':
            with open(file_path, 'r') as f:
                return np.array(json.load(f))
        raise ValueError(f"Unsupported grid format: {ext}")

    # -- Coordinate helpers ------------------------------------------------

    def _world_to_grid(self, point):
        gx = int((point[0] - self.x_range[0]) / self.resolution)
        gy = int((point[1] - self.y_range[0]) / self.resolution)
        return gx, gy

    def _is_free(self, point):
        gx, gy = self._world_to_grid(point)
        return (0 <= gx < self.cols and 0 <= gy < self.rows
                and self.grid[gy, gx] == 0)

    def _line_collision_free(self, p1, p2):
        for i in range(self.line_check_samples + 1):
            t = i / self.line_check_samples
            pt = p1 + t * (p2 - p1)
            if not self._is_free(pt):
                return False
        return True

    # -- Particle representation -------------------------------------------
    # A particle's position is a flat array of shape (n_waypoints * 2,)
    # representing [x0, y0, x1, y1, ..., x_{n-1}, y_{n-1}].

    def _decode_path(self, position):
        """Convert flat position vector to list of (x, y) waypoints
        including start and goal."""
        pts = position.reshape(-1, 2)
        path = [self.start.copy()]
        for p in pts:
            path.append(p.copy())
        path.append(self.goal.copy())
        return path

    def _fitness(self, position):
        """Evaluate fitness: total path length + collision penalty."""
        path = self._decode_path(position)
        total_length = 0.0
        penalty = 0.0
        for i in range(len(path) - 1):
            seg_len = np.linalg.norm(path[i + 1] - path[i])
            total_length += seg_len
            if not self._line_collision_free(path[i], path[i + 1]):
                penalty += self.COLLISION_PENALTY
        return total_length + penalty

    # -- PSO core ----------------------------------------------------------

    def _run_pso(self):
        dim = self.n_waypoints * 2

        # Initialise positions: linear interpolation + noise
        positions = np.zeros((self.n_particles, dim))
        for i in range(self.n_particles):
            for w in range(self.n_waypoints):
                t = (w + 1) / (self.n_waypoints + 1)
                interp = self.start + t * (self.goal - self.start)
                noise_x = self._rng.uniform(-8.0, 8.0)
                noise_y = self._rng.uniform(-8.0, 8.0)
                positions[i, 2 * w] = np.clip(
                    interp[0] + noise_x, self.x_min, self.x_max)
                positions[i, 2 * w + 1] = np.clip(
                    interp[1] + noise_y, self.y_min, self.y_max)

        velocities = self._rng.uniform(-1.0, 1.0, (self.n_particles, dim))

        # Evaluate initial fitness
        fitness = np.array([self._fitness(p) for p in positions])
        pbest_pos = positions.copy()
        pbest_fit = fitness.copy()
        gbest_idx = np.argmin(pbest_fit)
        gbest_pos = pbest_pos[gbest_idx].copy()
        gbest_fit = pbest_fit[gbest_idx]

        # Record initial state
        self._history.append((
            positions.copy(),
            self._decode_path(gbest_pos),
            float(gbest_fit),
        ))

        for it in range(self.max_iter):
            r1 = self._rng.random((self.n_particles, dim))
            r2 = self._rng.random((self.n_particles, dim))

            # Update velocities
            velocities = (self.w * velocities
                          + self.c1 * r1 * (pbest_pos - positions)
                          + self.c2 * r2 * (gbest_pos - positions))

            # Update positions
            positions += velocities

            # Clamp to world bounds
            for w in range(self.n_waypoints):
                positions[:, 2 * w] = np.clip(
                    positions[:, 2 * w], self.x_min, self.x_max)
                positions[:, 2 * w + 1] = np.clip(
                    positions[:, 2 * w + 1], self.y_min, self.y_max)

            # Evaluate fitness
            fitness = np.array([self._fitness(p) for p in positions])

            # Update personal bests
            improved = fitness < pbest_fit
            pbest_pos[improved] = positions[improved]
            pbest_fit[improved] = fitness[improved]

            # Update global best
            gen_best = np.argmin(pbest_fit)
            if pbest_fit[gen_best] < gbest_fit:
                gbest_pos = pbest_pos[gen_best].copy()
                gbest_fit = pbest_fit[gen_best]

            # Record history (subsample for animation)
            if it % 2 == 0 or it == self.max_iter - 1:
                self._history.append((
                    positions.copy(),
                    self._decode_path(gbest_pos),
                    float(gbest_fit),
                ))

        # Final path
        path_arrays = self._decode_path(gbest_pos)
        self.path = [tuple(p) for p in path_arrays]

        collision_free = all(
            self._line_collision_free(
                np.array(self.path[i]), np.array(self.path[i + 1]))
            for i in range(len(self.path) - 1)
        )
        print(f"PSO: converged after {self.max_iter} iterations, "
              f"fitness={gbest_fit:.2f}, "
              f"collision_free={collision_free}, "
              f"waypoints={len(self.path)}")

    # -- Path utilities ----------------------------------------------------

    def _make_sparse_path(self, path, num_points=20):
        if len(path) <= num_points:
            return [list(p) for p in path]
        indices = np.linspace(0, len(path) - 1, num_points, dtype=int)
        return [list(path[i]) for i in indices]

    def _save_path(self, path, filename):
        Path(filename).parent.mkdir(parents=True, exist_ok=True)
        with open(filename, 'w') as f:
            json.dump(path, f)

    # -- Visualisation -----------------------------------------------------

    def visualize_search(self, gif_name=None):
        """
        Render a GIF showing PSO convergence:
          Phase 0: Swarm iterations (particles + current global best path)
          Phase 1: Final path drawing
          Phase 2: Hold final
        """
        if gif_name is None:
            return
        if not self._history:
            return

        # Colour map for grid
        cmap = ListedColormap([
            [1.0, 1.0, 1.0],    # free
            [0.5, 0.5, 0.5],    # clearance
            [0.0, 0.0, 0.0],    # obstacle
        ])

        def _disc(g):
            out = np.zeros_like(g, dtype=int)
            out[np.isclose(g, 0.75)] = 1
            out[g >= 0.99] = 2
            return out

        grid_disc = _disc(self.grid)

        # Frame plan
        n_hist = len(self._history)
        path_frames = 20
        hold_frames = 15
        phase_lens = [n_hist, path_frames, hold_frames]
        offsets = np.cumsum([0] + phase_lens)
        total = int(offsets[-1])

        def _phase(i):
            for p in range(3):
                if i < offsets[p + 1]:
                    return p, i - int(offsets[p])
            return 2, hold_frames - 1

        def update(i, ax):
            phase, local = _phase(i)
            ax.clear()

            # Draw grid
            ax.imshow(grid_disc,
                      extent=[self.x_range[0], self.x_range[-1],
                              self.y_range[0], self.y_range[-1]],
                      origin='lower', cmap=cmap, vmin=0, vmax=2,
                      alpha=0.8)

            if phase == 0:
                snap_idx = min(local, n_hist - 1)
                positions, gbest_path, gfit = self._history[snap_idx]

                # Draw all particles' waypoints
                for p_idx in range(positions.shape[0]):
                    pts = self._decode_path(positions[p_idx])
                    px = [pt[0] for pt in pts]
                    py = [pt[1] for pt in pts]
                    ax.plot(px, py, '-', color='#90CAF9', linewidth=0.5,
                            alpha=0.3, zorder=2)
                    ax.scatter(px[1:-1], py[1:-1], c='#42A5F5', s=6,
                               alpha=0.4, zorder=3)

                # Draw global best path
                if gbest_path:
                    gx = [pt[0] for pt in gbest_path]
                    gy = [pt[1] for pt in gbest_path]
                    ax.plot(gx, gy, '-', color='#2E7D32', linewidth=2.0,
                            zorder=5, label=f"Best (L={gfit:.1f})")
                    ax.scatter(gx[1:-1], gy[1:-1], c='#2E7D32', s=20,
                               zorder=6)

                iter_num = snap_idx * 2 if snap_idx < n_hist - 1 else self.max_iter
                ax.set_title(
                    f"PSO — Iteration {iter_num}/{self.max_iter}",
                    fontsize=14)

            elif phase >= 1:
                # Draw final path
                if self.path:
                    if phase == 1:
                        frac = min(local + 1, path_frames)
                        n = max(1, int(len(self.path)
                                       * frac / path_frames))
                    else:
                        n = len(self.path)
                    seg = self.path[:n]
                    if len(seg) > 1:
                        px = [p[0] for p in seg]
                        py = [p[1] for p in seg]
                        ax.plot(px, py, '-', color='#2E7D32',
                                linewidth=2.5, zorder=5, label="Path")
                        ax.scatter(px[1:-1], py[1:-1], c='#2E7D32',
                                   s=25, zorder=6)

                ax.set_title("PSO — Optimised Path", fontsize=14)

            # Start and goal
            ax.plot(self.start[0], self.start[1], 'go', markersize=10,
                    label="Start", zorder=7)
            ax.plot(self.goal[0], self.goal[1], 'ro', markersize=10,
                    label="Goal", zorder=7)

            ax.legend(loc='upper left')
            ax.set_xlabel("X [m]", fontsize=12)
            ax.set_ylabel("Y [m]", fontsize=12)
            ax.set_aspect("equal")

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111)
        ax.set_aspect("equal")

        print(f"PSO search animation: {total} frames")
        anime = anm.FuncAnimation(fig, update, fargs=(ax,),
                                  frames=total, interval=50, repeat=False)
        try:
            anime.save(gif_name, writer="pillow", fps=20)
            print(f"Search GIF saved to {gif_name}")
        except Exception as e:
            print(f"Error saving search GIF: {e}")
        plt.clf()
        plt.close()


if __name__ == "__main__":
    map_file = "map.json"
    path_file = "path.json"
    gif_path = "pso_search.gif"

    x_lim, y_lim = MinMax(-5, 55), MinMax(-20, 25)
    start = (0, 0)
    goal = (50, -10)

    planner = PsoPathPlanner(
        start, goal, map_file,
        x_lim=x_lim, y_lim=y_lim,
        path_filename=path_file,
        gif_name=gif_path,
    )

"""
pso_path_planner.py

Particle Swarm Optimization (PSO) path planner.

Each particle is a 2-D agent that navigates from the start region toward
the goal.  At every iteration each particle updates its velocity via the
classic PSO equation (inertia + cognitive + social terms) and takes a
step.  If the step would collide with an obstacle the velocity is
deflected and reduced so the particle slides around it.

The **global-best particle's trail** of positions forms the planned path.
Once the global best reaches the goal the search terminates early.

The search GIF shows the swarm swarming through the map: each particle
leaves a thin trail so you can see the exploration; colliding steps are
coloured red while free ones are blue; and the global-best trail is drawn
prominently in green.

References:
    Kennedy & Eberhart, "Particle Swarm Optimization", IEEE ICNN, 1995.
    Shi & Eberhart, "A Modified Particle Swarm Optimizer", IEEE WCCI, 1998.

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


# -----------------------------------------------------------------------
# Particle
# -----------------------------------------------------------------------


class _Particle:
    """A single particle (agent) in the swarm."""

    def __init__(self, position, max_vel, rng):
        self.pos = np.array(position, dtype=float)
        self.vel = rng.uniform(-0.3, 0.3, size=2)
        self.max_vel = max_vel
        self.pbest_pos = self.pos.copy()
        self.pbest_val = np.inf
        self.trail = [self.pos.copy()]


# -----------------------------------------------------------------------
# Public planner
# -----------------------------------------------------------------------


class PsoPathPlanner:
    """
    PSO swarm-navigation path planner on a 2-D occupancy grid.

    Particles start near *start* and swarm toward *goal*.  The fitness
    of a particle is its Euclidean distance to the goal plus a penalty
    for proximity to obstacles.  The global-best particle's trail is
    the planned path.

    Constructor parameters follow the project convention used by
    ``AStarPathPlanner``, ``RrtPathPlanner``, etc.
    """

    # Proximity penalty parameters
    _OBSTACLE_PENALTY = 500.0
    _PROXIMITY_RADIUS = 2.0  # world-units; soft penalty zone around obstacles
    _GOAL_TOLERANCE = 2.0  # world-units; "close enough" to goal

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
        n_particles=30,
        max_iter=300,
        w_start=0.9,
        w_end=0.4,
        c1=1.8,
        c2=2.0,
        max_vel=3.0,
        seed=42,
    ):
        self.start = np.array(start, dtype=float)
        self.goal = np.array(goal, dtype=float)
        self.n_particles = n_particles
        self.max_iter = max_iter
        self.w_start = w_start
        self.w_end = w_end
        self.c1 = c1
        self.c2 = c2
        self.max_vel = max_vel

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

        # Run
        self.path = []
        self._history = []  # per-snapshot data for animation
        self._run()

        # Save
        if path_filename and self.path:
            sparse = self._make_sparse_path(self.path)
            self._save_path(sparse, path_filename)

        self.visualize_search(gif_name)

    # -- Grid I/O ----------------------------------------------------------

    @staticmethod
    def _load_grid(file_path):
        ext = Path(file_path).suffix
        if ext == ".npy":
            return np.load(file_path)
        if ext == ".png":
            g = plt.imread(file_path)
            if g.ndim == 3:
                g = np.mean(g, axis=2)
            return (g > 0.5).astype(float)
        if ext == ".json":
            with open(file_path, "r") as f:
                return np.array(json.load(f))
        raise ValueError(f"Unsupported grid format: {ext}")

    # -- Coordinate helpers ------------------------------------------------

    def _world_to_grid(self, pt):
        gx = int((pt[0] - self.x_range[0]) / self.resolution)
        gy = int((pt[1] - self.y_range[0]) / self.resolution)
        return gx, gy

    def _is_free(self, pt):
        gx, gy = self._world_to_grid(pt)
        return 0 <= gx < self.cols and 0 <= gy < self.rows and self.grid[gy, gx] == 0

    def _segment_free(self, p1, p2, n_checks=10):
        """Check collision along a line segment."""
        for i in range(n_checks + 1):
            t = i / n_checks
            if not self._is_free(p1 + t * (p2 - p1)):
                return False
        return True

    def _obstacle_proximity_penalty(self, pt):
        """Soft penalty: the closer to an obstacle the higher the cost."""
        gx, gy = self._world_to_grid(pt)
        if not (0 <= gx < self.cols and 0 <= gy < self.rows):
            return self._OBSTACLE_PENALTY
        if self.grid[gy, gx] > 0:
            return self._OBSTACLE_PENALTY
        # Check neighbourhood in grid for nearby obstacles
        r_cells = int(self._PROXIMITY_RADIUS / self.resolution) + 1
        min_dist = self._PROXIMITY_RADIUS
        for dy in range(-r_cells, r_cells + 1):
            for dx in range(-r_cells, r_cells + 1):
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < self.cols and 0 <= ny < self.rows:
                    if self.grid[ny, nx] > 0:
                        d = np.sqrt(dx * dx + dy * dy) * self.resolution
                        if d < min_dist:
                            min_dist = d
        if min_dist < self._PROXIMITY_RADIUS:
            return 15.0 / (min_dist + 0.1)
        return 0.0

    # -- Fitness -----------------------------------------------------------

    def _fitness(self, pos):
        """Distance to goal + obstacle proximity penalty."""
        dist = np.linalg.norm(pos - self.goal)
        return dist + self._obstacle_proximity_penalty(pos)

    # -- PSO core ----------------------------------------------------------

    def _run(self):
        # Spawn particles spread around start so the swarm fans out
        spawn_radius = 15.0
        particles = []
        for _ in range(self.n_particles):
            sx = self.start[0] + self._rng.uniform(-spawn_radius, spawn_radius)
            sy = self.start[1] + self._rng.uniform(-spawn_radius, spawn_radius)
            p = _Particle([sx, sy], self.max_vel, self._rng)
            p.pbest_val = self._fitness(p.pos)
            particles.append(p)

        # Global best
        gbest_pos = particles[0].pos.copy()
        gbest_val = particles[0].pbest_val
        for p in particles:
            if p.pbest_val < gbest_val:
                gbest_val = p.pbest_val
                gbest_pos = p.pos.copy()
        gbest_trail = [self.start.copy(), gbest_pos.copy()]

        reached_goal = False

        for it in range(self.max_iter):
            # Linearly decay inertia
            w = self.w_start - (self.w_start - self.w_end) * it / self.max_iter

            for p in particles:
                r1 = self._rng.random(2)
                r2 = self._rng.random(2)
                p.vel = (
                    w * p.vel
                    + self.c1 * r1 * (p.pbest_pos - p.pos)
                    + self.c2 * r2 * (gbest_pos - p.pos)
                )
                # Clamp velocity
                speed = np.linalg.norm(p.vel)
                if speed > p.max_vel:
                    p.vel = p.vel / speed * p.max_vel

                next_pos = p.pos + p.vel

                # Clamp to world bounds
                next_pos[0] = np.clip(next_pos[0], self.x_min, self.x_max)
                next_pos[1] = np.clip(next_pos[1], self.y_min, self.y_max)

                # Collision handling: if next position collides, deflect
                if not self._segment_free(p.pos, next_pos):
                    p.vel *= -0.3  # bounce back weakly
                    next_pos = p.pos + p.vel
                    next_pos[0] = np.clip(next_pos[0], self.x_min, self.x_max)
                    next_pos[1] = np.clip(next_pos[1], self.y_min, self.y_max)
                    if not self._is_free(next_pos):
                        next_pos = p.pos.copy()  # stay put

                p.pos = next_pos
                p.trail.append(p.pos.copy())

                # Update personal best
                val = self._fitness(p.pos)
                if val < p.pbest_val:
                    p.pbest_val = val
                    p.pbest_pos = p.pos.copy()

                # Update global best
                if val < gbest_val:
                    gbest_val = val
                    gbest_pos = p.pos.copy()

            gbest_trail.append(gbest_pos.copy())

            # Check if global best reached goal
            if np.linalg.norm(gbest_pos - self.goal) < self._GOAL_TOLERANCE:
                reached_goal = True

            # Record every iteration so the GIF shows each step
            if True:
                self._history.append(
                    {
                        "positions": np.array([p.pos.copy() for p in particles]),
                        "trails": [list(p.trail) for p in particles],
                        "gbest_trail": list(gbest_trail),
                        "gbest_val": float(gbest_val),
                        "iteration": it + 1,
                    }
                )

            if reached_goal:
                print(f"PSO: goal reached at iteration {it + 1}")
                break

        # Build final path from global-best trail
        # Prepend start, append goal, deduplicate
        raw = [self.start.copy()] + gbest_trail + [self.goal.copy()]
        # Simplify: keep only waypoints that are far enough apart
        simplified = [raw[0]]
        for pt in raw[1:]:
            if np.linalg.norm(pt - simplified[-1]) > 0.3:
                simplified.append(pt)
        if np.linalg.norm(simplified[-1] - self.goal) > 0.1:
            simplified.append(self.goal.copy())
        self.path = [tuple(p) for p in simplified]

        print(
            f"PSO: {self.max_iter if not reached_goal else it + 1} iterations, "
            f"fitness={gbest_val:.2f}, "
            f"reached_goal={reached_goal}, "
            f"path_len={len(self.path)}"
        )

    # -- Path utilities ----------------------------------------------------

    def _make_sparse_path(self, path, num_points=20):
        if len(path) <= num_points:
            return [list(p) for p in path]
        indices = np.linspace(0, len(path) - 1, num_points, dtype=int)
        return [list(path[i]) for i in indices]

    def _save_path(self, path, filename):
        Path(filename).parent.mkdir(parents=True, exist_ok=True)
        with open(filename, "w") as f:
            json.dump(path, f)

    # -- Visualisation -----------------------------------------------------

    def visualize_search(self, gif_name=None):
        """
        Render a GIF showing the PSO swarm navigating from start to goal.

        Phase 0 — Swarm iterations:
          - Each particle drawn as a dot with a fading trail behind it
          - Blue dots/trails = collision-free; red = colliding
          - Global best trail drawn prominently in green
          - Title shows iteration, fitness, and particle count

        Phase 1 — Final path drawn progressively

        Phase 2 — Hold
        """
        if gif_name is None:
            return
        if not self._history:
            return

        cmap = ListedColormap(
            [
                [1.0, 1.0, 1.0],  # free
                [0.75, 0.75, 0.75],  # clearance
                [0.15, 0.15, 0.15],  # obstacle
            ]
        )

        def _disc(g):
            out = np.zeros_like(g, dtype=int)
            out[np.isclose(g, 0.75)] = 1
            out[g >= 0.99] = 2
            return out

        grid_disc = _disc(self.grid)

        # Colours
        _C_FREE = "#42A5F5"
        _C_COLL = "#EF5350"
        _C_BEST = "#2E7D32"
        _C_TRAIL = "#90CAF9"
        _C_TRAIL_BAD = "#EF9A9A"

        n_hist = len(self._history)
        path_frames = 25
        hold_frames = 15
        offsets = np.cumsum([0, n_hist, path_frames, hold_frames])
        total = int(offsets[-1])

        def _phase(i):
            for p in range(3):
                if i < offsets[p + 1]:
                    return p, i - int(offsets[p])
            return 2, hold_frames - 1

        def _draw_grid(ax):
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

        max_trail_len = 30  # only draw last N steps of each trail

        def update(i, ax):
            phase, local = _phase(i)
            ax.clear()
            _draw_grid(ax)

            if phase == 0:
                snap = self._history[min(local, n_hist - 1)]
                positions = snap["positions"]
                trails = snap["trails"]
                gbest_trail = snap["gbest_trail"]
                gval = snap["gbest_val"]
                it_num = snap["iteration"]

                # Draw particle trails and current positions
                for t_idx, trail in enumerate(trails):
                    if len(trail) < 2:
                        continue
                    # Trim to last max_trail_len points
                    segment = trail[-max_trail_len:]
                    tx = [pt[0] for pt in segment]
                    ty = [pt[1] for pt in segment]
                    # Check if current position is free
                    cur = positions[t_idx]
                    free = self._is_free(cur)
                    tcol = _C_TRAIL if free else _C_TRAIL_BAD
                    dcol = _C_FREE if free else _C_COLL
                    ax.plot(
                        tx, ty, "-", color=tcol, linewidth=0.5, alpha=0.35, zorder=2
                    )
                    ax.plot(
                        cur[0],
                        cur[1],
                        "o",
                        color=dcol,
                        markersize=4,
                        alpha=0.7,
                        zorder=4,
                    )

                # Global best trail
                if len(gbest_trail) > 1:
                    gx = [pt[0] for pt in gbest_trail]
                    gy = [pt[1] for pt in gbest_trail]
                    ax.plot(
                        gx,
                        gy,
                        "-",
                        color=_C_BEST,
                        linewidth=2.0,
                        alpha=0.8,
                        zorder=5,
                        label="Best Trail",
                    )
                    # Current best position
                    ax.plot(
                        gx[-1],
                        gy[-1],
                        "o",
                        color=_C_BEST,
                        markersize=8,
                        markeredgecolor="white",
                        markeredgewidth=1.0,
                        zorder=6,
                    )

                ax.set_title(
                    f"PSO — Iter {it_num}/{self.max_iter}  |  " f"fitness = {gval:.1f}",
                    fontsize=13,
                )
                # Legend proxies
                ax.plot(
                    [], [], "o", color=_C_FREE, markersize=5, label="Particle (free)"
                )
                ax.plot(
                    [], [], "o", color=_C_COLL, markersize=5, label="Particle (blocked)"
                )
                ax.plot([], [], "-", color=_C_BEST, linewidth=2, label="Global Best")

            elif phase >= 1:
                # Draw final path
                if self.path:
                    if phase == 1:
                        frac = min(local + 1, path_frames)
                        n = max(1, int(len(self.path) * frac / path_frames))
                    else:
                        n = len(self.path)
                    seg = self.path[:n]
                    if len(seg) > 1:
                        px = [p[0] for p in seg]
                        py = [p[1] for p in seg]
                        ax.plot(
                            px,
                            py,
                            "-",
                            color=_C_BEST,
                            linewidth=3.0,
                            zorder=5,
                            label="Path",
                        )
                ax.set_title("PSO — Final Path", fontsize=14)

            # Endpoints
            ax.plot(
                self.start[0],
                self.start[1],
                "o",
                color=_C_BEST,
                markersize=11,
                markeredgecolor="white",
                markeredgewidth=1.2,
                label="Start",
                zorder=8,
            )
            ax.plot(
                self.goal[0],
                self.goal[1],
                "o",
                color=_C_COLL,
                markersize=11,
                markeredgecolor="white",
                markeredgewidth=1.2,
                label="Goal",
                zorder=8,
            )

            ax.legend(loc="upper left", fontsize=9, framealpha=0.85)
            ax.set_xlabel("X [m]", fontsize=12)
            ax.set_ylabel("Y [m]", fontsize=12)
            ax.set_aspect("equal")

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111)
        ax.set_aspect("equal")

        print(f"PSO search animation: {total} frames")
        anime = anm.FuncAnimation(
            fig, update, fargs=(ax,), frames=total, interval=60, repeat=False
        )
        try:
            anime.save(gif_name, writer="pillow", fps=8)
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
        start,
        goal,
        map_file,
        x_lim=x_lim,
        y_lim=y_lim,
        path_filename=path_file,
        gif_name=gif_path,
    )

"""
elastic_bands_path_planner.py

Elastic Bands path planner.

Uses A* to find an initial path on an occupancy grid, then smooths it
with the Elastic Bands algorithm (Quinlan & Khatib, 1993).

Each waypoint is wrapped in a "bubble" whose radius equals the distance
to the nearest obstacle (from a signed-distance field).  Internal
contraction forces pull the band taut while external repulsive forces
push it away from obstacles.  An overlap constraint maintains connectivity
by inserting / deleting bubbles as needed.

Reference:
    Elastic Bands: Connecting Path Planning and Control
    http://www8.cs.umu.se/research/ifor/dl/Control/elastic%20bands.pdf

Author: Wang Zheng (@Aglargil), adapted to project architecture
"""

import numpy as np
import heapq
import json
import matplotlib.pyplot as plt
import matplotlib.animation as anm
import sys
from pathlib import Path
from matplotlib.colors import ListedColormap
from matplotlib.patches import Circle
from scipy.ndimage import distance_transform_edt

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"
relative_simulations = "/../../../simulations/"

sys.path.append(abs_dir_path + relative_path + "visualization")
sys.path.append(abs_dir_path + relative_path + "state")
sys.path.append(abs_dir_path + relative_path + "obstacle")
sys.path.append(abs_dir_path + relative_path + "mapping/grid")

from min_max import MinMax  # noqa: E402


# ---------------------------------------------------------------------------
# Bubble
# ---------------------------------------------------------------------------

class _Bubble:
    """A single bubble in the elastic band."""

    MAX_RADIUS = 20.0
    MIN_RADIUS = 0.5

    def __init__(self, position, radius):
        self.pos = np.array(position, dtype=float)
        self.radius = float(np.clip(radius, self.MIN_RADIUS, self.MAX_RADIUS))


# ---------------------------------------------------------------------------
# Core Elastic Bands optimiser (grid-index space)
# ---------------------------------------------------------------------------

class _ElasticBandsOptimiser:
    """
    Operates entirely in **grid-index** space.

    Parameters
    ----------
    initial_path : list[tuple[int,int]]
        Waypoints as (col, row) grid indices.
    sdf : np.ndarray
        Signed-distance field (in grid cells) — same shape as the grid.
        ``sdf[row, col]`` gives the distance to the nearest obstacle.
    rho0 : float
        Maximum distance for applying repulsive force.
    kc, kr : float
        Contraction / repulsive force gains.
    lambda_ : float
        Overlap constraint factor.
    step_size : float
        Finite-difference step for SDF gradient.
    max_iter : int
        Maximum optimisation iterations.
    """

    def __init__(self, initial_path, sdf, *,
                 rho0=5.0, kc=0.3, kr=-0.05, lambda_=0.6,
                 step_size=1.0, max_iter=60):
        self.sdf = sdf
        self.rows, self.cols = sdf.shape
        self.rho0 = rho0
        self.kc = kc
        self.kr = kr
        self.lambda_ = lambda_
        self.step_size = step_size
        self.max_iter = max_iter

        # Build initial bubble chain
        self.bubbles = [
            _Bubble(p, self._rho(p)) for p in initial_path
        ]
        self._maintain_overlap()

        # Snapshot history for animation (list of bubble-chain snapshots)
        self.history: list[list[_Bubble]] = [self._snapshot()]

    # -- SDF helpers -------------------------------------------------------

    def _rho(self, pos):
        """Distance to nearest obstacle at *pos* (col, row)."""
        c, r = int(round(pos[0])), int(round(pos[1]))
        c = np.clip(c, 0, self.cols - 1)
        r = np.clip(r, 0, self.rows - 1)
        return float(self.sdf[r, c])

    # -- Forces ------------------------------------------------------------

    def _contraction_force(self, i):
        if i == 0 or i == len(self.bubbles) - 1:
            return np.zeros(2)
        prev = self.bubbles[i - 1].pos
        nxt = self.bubbles[i + 1].pos
        cur = self.bubbles[i].pos
        d_prev = prev - cur
        d_next = nxt - cur
        n_prev = np.linalg.norm(d_prev) + 1e-9
        n_next = np.linalg.norm(d_next) + 1e-9
        return self.kc * (d_prev / n_prev + d_next / n_next)

    def _repulsive_force(self, i):
        b = self.bubbles[i].pos
        rho = self.bubbles[i].radius
        if rho >= self.rho0:
            return np.zeros(2)
        h = self.step_size
        dx = np.array([h, 0.0])
        dy = np.array([0.0, h])
        grad_x = (self._rho(b - dx) - self._rho(b + dx)) / (2 * h)
        grad_y = (self._rho(b - dy) - self._rho(b + dy)) / (2 * h)
        grad = np.array([grad_x, grad_y])
        return self.kr * (self.rho0 - rho) * grad

    # -- Update step -------------------------------------------------------

    def _update_bubbles(self):
        new = []
        for i, bub in enumerate(self.bubbles):
            if i == 0 or i == len(self.bubbles) - 1:
                new.append(bub)
                continue
            f_total = self._contraction_force(i) + self._repulsive_force(i)
            v = self.bubbles[i - 1].pos - self.bubbles[i + 1].pos
            v_norm2 = np.dot(v, v) + 1e-9
            # Remove tangential component (project onto normal)
            f_star = f_total - (np.dot(f_total, v) / v_norm2) * v
            alpha = min(bub.radius, 3.0)  # adaptive step, clamped
            new_pos = bub.pos + alpha * f_star
            new_pos[0] = np.clip(new_pos[0], 0, self.cols - 1)
            new_pos[1] = np.clip(new_pos[1], 0, self.rows - 1)
            new.append(_Bubble(new_pos, self._rho(new_pos)))
        self.bubbles = new

    # -- Overlap maintenance -----------------------------------------------

    def _maintain_overlap(self):
        # Insert
        i = 0
        while i < len(self.bubbles) - 1:
            bi, bj = self.bubbles[i], self.bubbles[i + 1]
            dist = np.linalg.norm(bi.pos - bj.pos)
            if dist > self.lambda_ * (bi.radius + bj.radius):
                mid = (bi.pos + bj.pos) / 2
                self.bubbles.insert(i + 1, _Bubble(mid, self._rho(mid)))
                i += 2
            else:
                i += 1
        # Delete redundant
        i = 1
        while i < len(self.bubbles) - 1:
            prev, nxt = self.bubbles[i - 1], self.bubbles[i + 1]
            if np.linalg.norm(prev.pos - nxt.pos) <= self.lambda_ * (prev.radius + nxt.radius):
                del self.bubbles[i]
            else:
                i += 1

    # -- Snapshot ----------------------------------------------------------

    def _snapshot(self):
        return [_Bubble(b.pos.copy(), b.radius) for b in self.bubbles]

    # -- Run ---------------------------------------------------------------

    def optimise(self):
        """Run the full optimisation loop, recording history."""
        for _ in range(self.max_iter):
            self._update_bubbles()
            self._maintain_overlap()
            self.history.append(self._snapshot())


# ---------------------------------------------------------------------------
# Public planner (follows project API)
# ---------------------------------------------------------------------------

class ElasticBandsPathPlanner:
    """
    Elastic Bands path planner operating on a 2-D occupancy grid.

    1. Loads the grid and computes a signed-distance field.
    2. Runs A* to obtain an initial path.
    3. Optimises the path with the Elastic Bands algorithm.
    4. Optionally saves a search/optimisation GIF.

    Constructor parameters match the project convention used by
    ``AStarPathPlanner``, ``DijkstraPathPlanner``, etc.
    """

    def __init__(self, start, goal, map_file, *,
                 x_lim=None, y_lim=None,
                 path_filename=None, gif_name=None,
                 max_iter=60):
        self.start = start
        self.goal = goal
        self.explored_nodes = []

        # Load grid
        self.grid = self._load_grid(map_file)
        x_min, x_max = x_lim.min_value(), x_lim.max_value()
        y_min, y_max = y_lim.min_value(), y_lim.max_value()
        self.resolution = (x_max - x_min) / self.grid.shape[1]
        self.x_range = np.arange(x_min, x_max, self.resolution)
        self.y_range = np.arange(y_min, y_max, self.resolution)
        self.rows, self.cols = self.grid.shape

        # SDF: distance (in cells) to nearest obstacle for every free cell
        obstacle_mask = (self.grid >= 0.99)
        self.sdf = distance_transform_edt(~obstacle_mask)

        # A* initial path
        self.path = []
        self._astar_path = self._astar_search()
        if not self._astar_path:
            print("Elastic Bands: A* found no initial path.")
            return

        # Elastic bands optimisation
        self._optimiser = _ElasticBandsOptimiser(
            self._astar_path, self.sdf, max_iter=max_iter,
        )
        self._optimiser.optimise()

        # Final smoothed path (grid indices)
        self.path = [(int(round(b.pos[0])), int(round(b.pos[1])))
                      for b in self._optimiser.bubbles]

        # Save sparse path
        if path_filename:
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

    # -- A* seed path ------------------------------------------------------

    def _astar_search(self):
        """Run A* on the grid, return path as list of (col, row) tuples."""
        sx = int((self.start[0] - self.x_range[0]) / self.resolution)
        sy = int((self.start[1] - self.y_range[0]) / self.resolution)
        gx = int((self.goal[0] - self.x_range[0]) / self.resolution)
        gy = int((self.goal[1] - self.y_range[0]) / self.resolution)
        start_idx = (sx, sy)
        goal_idx = (gx, gy)

        print(f"Elastic Bands — A* seed  Start(grid): {start_idx}, "
              f"Goal(grid): {goal_idx}")

        open_list = []
        heapq.heappush(open_list, (0, start_idx))
        came_from = {}
        cost = {start_idx: 0}

        while open_list:
            _, current = heapq.heappop(open_list)
            self.explored_nodes.append(current)
            if current == goal_idx:
                return self._reconstruct(came_from, start_idx, goal_idx)
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1),
                           (1, 1), (-1, -1), (1, -1), (-1, 1)]:
                nb = (current[0] + dx, current[1] + dy)
                if not (0 <= nb[0] < self.cols and 0 <= nb[1] < self.rows):
                    continue
                if self.grid[nb[1], nb[0]] >= 0.99:
                    continue
                step = 1.414 if (dx != 0 and dy != 0) else 1.0
                nc = cost[current] + step
                if nb not in cost or nc < cost[nb]:
                    cost[nb] = nc
                    h = ((nb[0] - gx) ** 2 + (nb[1] - gy) ** 2) ** 0.5
                    heapq.heappush(open_list, (nc + h, nb))
                    came_from[nb] = current
        return []

    @staticmethod
    def _reconstruct(came_from, start, goal):
        path, cur = [], goal
        while cur != start:
            path.append(cur)
            cur = came_from[cur]
        path.append(start)
        return path[::-1]

    # -- Path utilities ----------------------------------------------------

    def _grid_to_world(self, node):
        return (self.x_range[0] + node[0] * self.resolution,
                self.y_range[0] + node[1] * self.resolution)

    def _make_sparse_path(self, path, num_points=20):
        if len(path) <= num_points:
            return [self._grid_to_world(p) for p in path]
        indices = np.linspace(0, len(path) - 1, num_points, dtype=int)
        return [self._grid_to_world(path[i]) for i in indices]

    def _save_path(self, path, filename):
        Path(filename).parent.mkdir(parents=True, exist_ok=True)
        with open(filename, 'w') as f:
            json.dump([list(p) for p in path], f)

    # -- Visualisation -----------------------------------------------------

    def visualize_search(self, gif_name=None):
        """Render a GIF showing A* search → initial path → EB optimisation."""
        if gif_name is None:
            return
        if not self._astar_path:
            return

        history = self._optimiser.history
        explored = self.explored_nodes

        # Sub-sample phases
        max_search = 80
        step_s = max(1, len(explored) // max_search)
        search_frames = list(range(0, len(explored), step_s))
        if search_frames[-1] != len(explored) - 1:
            search_frames.append(len(explored) - 1)

        max_opt = 60
        step_o = max(1, len(history) // max_opt)
        opt_indices = list(range(0, len(history), step_o))
        if opt_indices[-1] != len(history) - 1:
            opt_indices.append(len(history) - 1)

        path_draw_frames = 20
        hold_frames = 15

        total = (len(search_frames) + path_draw_frames + hold_frames
                 + len(opt_indices) + hold_frames)
        offsets = np.cumsum([0, len(search_frames), path_draw_frames,
                             hold_frames, len(opt_indices), hold_frames])

        def _phase(i):
            for p in range(5):
                if i < offsets[p + 1]:
                    return p, i - int(offsets[p])
            return 4, hold_frames - 1

        # Colour map
        cmap = ListedColormap([
            [1.0, 1.0, 1.0],    # 0  free
            [0.4, 0.8, 1.0],    # 1  explored
            [0.0, 0.8, 0.0],    # 2  A* path
            [0.78, 0.78, 0.78], # 3  old path (greyed)
            [0.5, 0.5, 0.5],    # 4  clearance
            [0.0, 0.0, 0.0],    # 5  obstacle
        ])

        base_grid = self.grid.copy()

        def _disc(d):
            out = np.zeros_like(d, dtype=int)
            out[d == 0] = 0
            out[np.isclose(d, 0.25)] = 1
            out[np.isclose(d, 0.5)] = 2
            out[np.isclose(d, 0.6)] = 3
            out[np.isclose(d, 0.75)] = 4
            out[d >= 0.99] = 5
            return out

        def _paint(disp, cells, val):
            for gx, gy in cells:
                if 0 <= gx < disp.shape[1] and 0 <= gy < disp.shape[0]:
                    if disp[gy, gx] < 0.99:
                        disp[gy, gx] = val

        titles = [
            lambda l: f"A* Initial Search ({search_frames[min(l, len(search_frames)-1)]+1}/{len(explored)})",
            lambda _: "Initial Path Found (A*)",
            lambda _: "Initial Path Found (A*)",
            lambda l: f"Elastic Bands Optimisation ({min(l+1, len(opt_indices))}/{len(opt_indices)})",
            lambda _: "Elastic Bands — Optimised Path",
        ]

        def update(i, ax):
            phase, local = _phase(i)
            disp = base_grid.copy()

            if phase == 0:
                idx = search_frames[min(local, len(search_frames) - 1)]
                _paint(disp, explored[:idx + 1], 0.25)
            elif phase == 1:
                _paint(disp, explored, 0.25)
                frac = min(local + 1, path_draw_frames)
                n = max(1, int(len(self._astar_path) * frac / path_draw_frames))
                _paint(disp, self._astar_path[:n], 0.5)
            elif phase == 2:
                _paint(disp, explored, 0.25)
                _paint(disp, self._astar_path, 0.5)
            elif phase == 3:
                # Grey out the A* path, show EB iteration
                _paint(disp, explored, 0.25)
                _paint(disp, self._astar_path, 0.6)
                snap = history[opt_indices[min(local, len(opt_indices) - 1)]]
                eb_cells = [(int(round(b.pos[0])), int(round(b.pos[1])))
                            for b in snap]
                _paint(disp, eb_cells, 0.5)
            else:
                _paint(disp, explored, 0.25)
                _paint(disp, self._astar_path, 0.6)
                _paint(disp, self.path, 0.5)

            ax.clear()
            ax.imshow(_disc(disp),
                      extent=[self.x_range[0], self.x_range[-1],
                              self.y_range[0], self.y_range[-1]],
                      origin='lower', cmap=cmap, vmin=0, vmax=5, alpha=0.85)
            ax.plot(self.start[0], self.start[1], 'go', markersize=8, label="Start")
            ax.plot(self.goal[0], self.goal[1], 'ro', markersize=8, label="Goal")

            # Draw bubbles during optimisation phase
            if phase >= 3:
                snap = (history[opt_indices[min(local, len(opt_indices) - 1)]]
                        if phase == 3 else self._optimiser.bubbles)
                for bub in snap:
                    wx = self.x_range[0] + bub.pos[0] * self.resolution
                    wy = self.y_range[0] + bub.pos[1] * self.resolution
                    r_world = bub.radius * self.resolution
                    circ = Circle((wx, wy), r_world, fill=False,
                                  color='green', alpha=0.25, linewidth=0.5)
                    ax.add_patch(circ)

            ax.set_title(titles[phase](local), fontsize=14)
            ax.legend(loc='upper left')

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111)
        ax.set_aspect("equal")
        ax.set_xlabel("X [m]", fontsize=15)
        ax.set_ylabel("Y [m]", fontsize=15)

        print(f"Elastic Bands search animation: {total} frames")
        anime = anm.FuncAnimation(fig, update, fargs=(ax,),
                                  frames=total, interval=30, repeat=False)
        try:
            anime.save(gif_name, writer="pillow", fps=20)
            print(f"Search GIF saved to {gif_name}")
        except Exception as e:
            print(f"Error saving search GIF: {e}")
        plt.clf()
        plt.close()

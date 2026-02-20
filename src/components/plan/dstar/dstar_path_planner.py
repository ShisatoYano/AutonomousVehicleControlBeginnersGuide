"""
dstar_path_planner.py

Implementation of the original D* (Dynamic A*) algorithm (Stentz, 1994)
with an optional focused heuristic (Focused D*).

D* searches backward from the goal and maintains cost-to-goal values for
every cell. When new obstacles are discovered on the current path, the
algorithm efficiently re-propagates costs only in the affected region
instead of re-planning from scratch.

By default, the priority queue is biased with a Euclidean heuristic toward
the robot (start) position, making the initial search informed (A*-like).
Set ``heuristic_weight=0`` to revert to an uninformed Dijkstra-style flood.

Key concepts:
    - Tag states: NEW (never visited), OPEN (on priority queue), CLOSED (processed).
    - Each cell stores h(X) = current cost-to-goal estimate.
    - The priority queue is keyed by k(X) + heuristic(X, robot), which
      focuses expansion toward the robot's position.
    - LOWER states propagate optimal costs outward.
    - RAISE states detect cost increases and attempt to find cheaper back-pointers.
"""

import numpy as np
import matplotlib.pyplot as plt
import heapq
import matplotlib.animation as anm
import sys
import json
from pathlib import Path
from matplotlib.colors import ListedColormap

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"
relative_simulations = "/../../../simulations/"

sys.path.append(abs_dir_path + relative_path + "visualization")
sys.path.append(abs_dir_path + relative_path + "state")
sys.path.append(abs_dir_path + relative_path + "obstacle")
sys.path.append(abs_dir_path + relative_path + "mapping/grid")

from min_max import MinMax


class _Tag:
    """Cell tag constants following Stentz's original D* paper."""
    NEW = 0
    OPEN = 1
    CLOSED = 2


class DStarPathPlanner:
    """
    Original D* path planner operating on a 2-D occupancy grid.

    The planner performs an initial backward search from the goal to the start.
    After the robot begins following the path, callers can inject new obstacles
    via ``update_obstacles`` which triggers efficient incremental replanning.

    Parameters
    ----------
    start : tuple[int, int]
        Start position in world coordinates (x, y).
    goal : tuple[int, int]
        Goal position in world coordinates (x, y).
    map_file : str
        Path to a ``.json``, ``.npy``, or ``.png`` grid file.
    x_lim : MinMax
        X-axis world limits.
    y_lim : MinMax
        Y-axis world limits.
    path_filename : str, optional
        If provided, the sparse path is saved here as JSON.
    gif_name : str, optional
        If provided, the search animation is saved here as a GIF.
    heuristic_weight : float, optional
        Weight for the Euclidean heuristic that biases the search toward
        the robot.  ``1.0`` (default) gives an A*-like informed search.
        ``0.0`` disables the heuristic (Dijkstra-style uniform flood).
        Values ``> 1.0`` are more aggressive but may over-expand during
        replanning.
    """

    # 8-connected neighbourhood (dx, dy, cost)
    _NEIGHBOURS = [
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (1, 1, 1.414),
    ]

    INF = float('inf')

    def __init__(self, start, goal, map_file, x_lim=None, y_lim=None,
                 path_filename=None, gif_name=None, heuristic_weight=1.0):
        self.start_world = start
        self.goal_world = goal
        self.heuristic_weight = heuristic_weight

        self.grid = self._load_grid(map_file)
        x_min, x_max = x_lim.min_value(), x_lim.max_value()
        y_min, y_max = y_lim.min_value(), y_lim.max_value()
        self.resolution = (x_max - x_min) / self.grid.shape[1]
        self.x_range = np.arange(x_min, x_max, self.resolution)
        self.y_range = np.arange(y_min, y_max, self.resolution)

        self.rows, self.cols = self.grid.shape

        # Convert world -> grid indices
        self.start_idx = self._world_to_grid(start)
        self.goal_idx = self._world_to_grid(goal)

        # The heuristic target: the search runs backward from goal, so the
        # heuristic biases expansion toward the robot (start) position.
        self._focus_target = self.start_idx

        # D* bookkeeping ---------------------------------------------------
        # tag[r,c]: NEW / OPEN / CLOSED
        self.tag = np.full((self.rows, self.cols), _Tag.NEW, dtype=np.int8)
        # h[r,c]: current cost-to-goal
        self.h = np.full((self.rows, self.cols), self.INF)
        # k[r,c]: key (priority) of the cell when on OPEN
        self.k = np.full((self.rows, self.cols), self.INF)
        # parent back-pointers (toward the goal)
        self.parent = {}  # (r,c) -> (pr,pc) or None

        # Priority queue: entries are (f_value, counter, (row, col))
        # where f = k + heuristic_weight * euclidean(node, focus_target)
        self._open_list = []
        self._counter = 0

        # Logging for visualisation
        self.explored_nodes = []
        self.replan_explored_nodes = []

        self.path = []
        self.path_filename = path_filename

        # ---------- initial search (goal -> start) ----------
        self._insert(self.goal_idx, 0.0)
        self.h[self.goal_idx] = 0.0
        self.parent[self.goal_idx] = None

        print(f"D* initial search  Start(grid): {self.start_idx}, Goal(grid): {self.goal_idx}"
              f"  heuristic_weight={self.heuristic_weight}")
        self._compute_shortest_path()

        # Extract the initial path
        self.path = self._extract_path()
        if path_filename and self.path:
            sparse = self._make_sparse_path(self.path)
            self._save_path(sparse, path_filename)

        self.visualize_search(gif_name)

    # ------------------------------------------------------------------
    #  Core D* operations (faithful to Stentz 1994)
    # ------------------------------------------------------------------

    def _heuristic(self, node):
        """Euclidean distance from *node* to the focus target (the robot).

        The D* search runs backward from the goal, so biasing toward the
        robot focuses the search and avoids flooding the entire grid.
        Returns 0 when ``heuristic_weight`` is 0 (Dijkstra mode).
        """
        if self.heuristic_weight == 0:
            return 0.0
        r, c = node
        tr, tc = self._focus_target
        return self.heuristic_weight * ((r - tr) ** 2 + (c - tc) ** 2) ** 0.5

    def _insert(self, node, h_new):
        """Insert or re-insert *node* into the OPEN list with updated cost.

        The heap key is ``k + heuristic(node)`` so that expansion is biased
        toward the robot.  The ``k`` value stored on the node remains
        un-biased so that the RAISE / LOWER logic (which compares ``k_old``
        against ``h``) is unaffected.
        """
        r, c = node
        if self.tag[r, c] == _Tag.NEW:
            k_val = h_new
        elif self.tag[r, c] == _Tag.OPEN:
            k_val = min(self.k[r, c], h_new)
        else:  # CLOSED
            k_val = min(self.h[r, c], h_new)
        self.k[r, c] = k_val
        self.h[r, c] = h_new
        self.tag[r, c] = _Tag.OPEN
        self._counter += 1
        f_val = k_val + self._heuristic(node)
        heapq.heappush(self._open_list, (f_val, self._counter, (r, c)))

    def _get_kmin(self):
        """Return the minimum key on the OPEN list (or -1 if empty)."""
        while self._open_list:
            k_val, _, node = self._open_list[0]
            r, c = node
            if self.tag[r, c] == _Tag.OPEN:
                return k_val
            heapq.heappop(self._open_list)  # stale entry
        return -1

    def _cost(self, a, b):
        """
        Arc cost c(a, b). Returns INF if *b* is an obstacle.
        a, b are (row, col) tuples.
        """
        br, bc = b
        if self.grid[br, bc] != 0:
            return self.INF
        ar, ac = a
        if self.grid[ar, ac] != 0:
            return self.INF
        # Euclidean distance between adjacent cells
        dr = abs(ar - br)
        dc = abs(ac - bc)
        if dr + dc == 2:
            return 1.414
        return 1.0

    def _neighbours(self, node):
        """Yield valid in-bounds neighbour (row, col) tuples."""
        r, c = node
        for dx, dy, _ in self._NEIGHBOURS:
            nr, nc = r + dy, c + dx
            if 0 <= nr < self.rows and 0 <= nc < self.cols:
                yield (nr, nc)

    def _process_state(self):
        """
        Pop the minimum-key OPEN state and propagate costs.

        Returns the k value of the processed state, or -1 if OPEN is empty.
        This follows Stentz's RAISE / LOWER propagation logic.

        Note: the heap stores ``f = k + heuristic`` for ordering, but the
        RAISE / LOWER logic uses the un-biased ``k`` value stored on the cell.
        """
        # Pop a valid OPEN entry
        while self._open_list:
            _f_val, _, node = heapq.heappop(self._open_list)
            r, c = node
            if self.tag[r, c] == _Tag.OPEN:
                break
        else:
            return -1  # OPEN list exhausted

        # Recover the real k (without heuristic) for RAISE/LOWER comparison
        k_old = self.k[r, c]

        self.tag[r, c] = _Tag.CLOSED
        self.explored_nodes.append((c, r))  # (grid_x, grid_y) for vis

        X = (r, c)
        h_X = self.h[r, c]

        # ---- CASE 1: RAISE (k_old < h(X))  ----
        # X's cost has increased.  Check if any neighbour Y can lower X.
        if k_old < h_X:
            for Y in self._neighbours(X):
                yr, yc = Y
                if self.tag[yr, yc] != _Tag.NEW and \
                   self.h[yr, yc] <= k_old and \
                   h_X > self.h[yr, yc] + self._cost(Y, X):
                    self.parent[X] = Y
                    self.h[r, c] = self.h[yr, yc] + self._cost(Y, X)
                    h_X = self.h[r, c]

        # ---- CASE 2: LOWER (k_old == h(X))  ----
        # X's cost is optimal.  Propagate to neighbours.
        if k_old == h_X:
            for Y in self._neighbours(X):
                yr, yc = Y
                c_YX = self._cost(X, Y)
                if self.tag[yr, yc] == _Tag.NEW or \
                   (self.parent.get(Y) == X and self.h[yr, yc] != h_X + c_YX) or \
                   (self.parent.get(Y) != X and self.h[yr, yc] > h_X + c_YX):
                    self.parent[Y] = X
                    self._insert(Y, h_X + c_YX)
        else:
            # ---- CASE 3: continued RAISE (k_old < h(X) still) ----
            for Y in self._neighbours(X):
                yr, yc = Y
                c_YX = self._cost(X, Y)

                if self.tag[yr, yc] == _Tag.NEW or \
                   (self.parent.get(Y) == X and self.h[yr, yc] != h_X + c_YX):
                    self.parent[Y] = X
                    self._insert(Y, h_X + c_YX)
                elif self.parent.get(Y) != X and self.h[yr, yc] > h_X + c_YX:
                    # X can improve Y, but re-insert X first so it will be
                    # processed as LOWER next time.
                    self._insert(X, h_X)
                elif self.parent.get(Y) != X and \
                     h_X > self.h[yr, yc] + self._cost(Y, X) and \
                     self.tag[yr, yc] == _Tag.CLOSED and \
                     self.h[yr, yc] > k_old:
                    # Y can potentially improve X, re-open Y.
                    self._insert(Y, self.h[yr, yc])

        return k_old

    def _compute_shortest_path(self):
        """Run process_state until the start is CLOSED or OPEN is empty."""
        sr, sc = self.start_idx
        while self.tag[sr, sc] != _Tag.CLOSED:
            if self._process_state() == -1:
                print("D*: OPEN list exhausted – no path found.")
                break

    # ------------------------------------------------------------------
    #  Dynamic replanning
    # ------------------------------------------------------------------

    def update_obstacles(self, obstacle_cells):
        """
        Notify the planner that new obstacles have appeared.

        Parameters
        ----------
        obstacle_cells : list[tuple[int, int]]
            Grid cells ``(row, col)`` that are now blocked.

        After calling this, call ``replan()`` to propagate costs.
        """
        for r, c in obstacle_cells:
            if not self._in_bounds(r, c):
                continue
            if self.grid[r, c] != 0:
                continue  # already an obstacle

            old_h = self.h[r, c]
            self.grid[r, c] = 1  # mark as obstacle

            # Re-insert the blocked cell with INF cost.
            # Its k will preserve the old value so it enters as a RAISE.
            self._insert((r, c), self.INF)

            # Any neighbour whose back-pointer goes through (r,c) is now
            # broken. Raise them as well.
            for Y in self._neighbours((r, c)):
                yr, yc = Y
                if self.parent.get(Y) == (r, c):
                    self._insert(Y, self.INF)

    def replan(self, current_pos_idx=None):
        """
        Run D* incremental repair until the robot's position is resolved.

        After ``update_obstacles`` places cells on the OPEN list, this method
        drains the OPEN list so that all RAISE / LOWER propagation completes.
        Then the parent chains are fully repaired and a new path can be
        extracted.

        Parameters
        ----------
        current_pos_idx : tuple[int, int], optional
            Current robot position in grid indices (row, col).
            Defaults to ``self.start_idx``.

        Returns
        -------
        list[tuple[int, int]]
            Updated path from the current position to the goal in grid indices.
        """
        if current_pos_idx is None:
            current_pos_idx = self.start_idx

        # Refocus the heuristic toward the robot's current position so
        # that replan expansion is biased toward where the robot is now.
        self._focus_target = current_pos_idx

        prev_len = len(self.explored_nodes)

        # Drain the OPEN list so all cost changes are propagated.
        while True:
            k_min = self._process_state()
            if k_min == -1:
                break

        self.replan_explored_nodes = self.explored_nodes[prev_len:]
        self.path = self._extract_path(start_idx=current_pos_idx)
        return self.path

    # ------------------------------------------------------------------
    #  Helpers
    # ------------------------------------------------------------------

    def _in_bounds(self, r, c):
        return 0 <= r < self.rows and 0 <= c < self.cols

    def _world_to_grid(self, world_xy):
        """Return ``(row, col)`` grid indices for a world ``(x, y)`` point."""
        x, y = world_xy
        col = int((x - self.x_range[0]) / self.resolution)
        row = int((y - self.y_range[0]) / self.resolution)
        return (row, col)

    def _grid_to_world(self, grid_rc):
        """Return ``(world_x, world_y)`` for a ``(row, col)`` grid index."""
        r, c = grid_rc
        wx = self.x_range[0] + c * self.resolution
        wy = self.y_range[0] + r * self.resolution
        return (wx, wy)

    def _extract_path(self, start_idx=None):
        """Follow parent pointers from *start_idx* to the goal."""
        if start_idx is None:
            start_idx = self.start_idx
        sr, sc = start_idx
        gr, gc = self.goal_idx

        if self.h[sr, sc] >= self.INF:
            print("D*: No path exists from start to goal.")
            return []

        path = [(sc, sr)]  # store as (grid_x, grid_y)
        current = (sr, sc)
        visited = set()
        while current != (gr, gc):
            if current in visited:
                print("D*: Cycle detected during path extraction.")
                return []
            visited.add(current)
            parent = self.parent.get(current)
            if parent is None:
                print("D*: Broken parent chain.")
                return []
            r, c = parent
            # If the parent is an obstacle, the chain is invalid
            if self.grid[r, c] != 0 and (r, c) != (gr, gc):
                print("D*: Parent chain goes through obstacle.")
                return []
            current = (r, c)
            path.append((c, r))
        return path

    def _make_sparse_path(self, path, num_points=20):
        if len(path) <= num_points:
            return [self._grid_to_world((r, c)) for c, r in path]
        indices = np.linspace(0, len(path) - 1, num_points, dtype=int)
        # path entries are (grid_x, grid_y), convert to world
        return [self._grid_to_world((path[i][1], path[i][0])) for i in indices]

    def _save_path(self, path, filename):
        Path(filename).parent.mkdir(parents=True, exist_ok=True)
        with open(filename, "w") as f:
            json.dump(path, f)

    @staticmethod
    def _load_grid(file_path):
        ext = Path(file_path).suffix
        if ext == '.npy':
            return np.load(file_path)
        if ext == '.png':
            grid = plt.imread(file_path)
            if grid.ndim == 3:
                grid = np.mean(grid, axis=2)
            return (grid > 0.5).astype(float)
        if ext == '.json':
            with open(file_path, 'r') as f:
                return np.array(json.load(f))
        raise ValueError(f"Unsupported grid format: {ext}")

    # ------------------------------------------------------------------
    #  Visualisation (matches the pattern used by A* / Dijkstra)
    # ------------------------------------------------------------------

    def visualize_search(self, gif_name=None):
        """Animate the initial search and, if present, the replan phase.

        When *gif_name* is ``None`` the method is a no-op so that callers
        (e.g. simulations that drive their own animation) can skip it.
        """
        print(f"D* explored {len(self.explored_nodes)} nodes during initial search.")
        if gif_name is None or not self.explored_nodes:
            return

        max_frames = 2000
        step = max(1, len(self.explored_nodes) // max_frames)
        sampled = self.explored_nodes[::step]
        self._sampled_nodes = sampled

        figure = plt.figure(figsize=(10, 8))
        axes = figure.add_subplot(111)
        axes.set_aspect("equal")
        axes.set_xlabel("X [m]", fontsize=15)
        axes.set_ylabel("Y [m]", fontsize=15)

        total_frames = len(sampled) + len(self.path)
        self.anime = anm.FuncAnimation(
            figure, self._update_frame, fargs=(axes,),
            frames=total_frames, interval=50, repeat=False,
        )

        try:
            print("Saving D* animation …")
            self.anime.save(gif_name, writer="pillow", fps=20)
            print("Animation saved successfully.")
        except Exception as e:
            print(f"Error saving animation: {e}")

        plt.clf()
        plt.close()

    def _update_frame(self, i, axes):
        display = self.grid.copy()
        sampled = self._sampled_nodes

        if i < len(sampled):
            for j in range(i + 1):
                gx, gy = sampled[j]
                if 0 <= gx < display.shape[1] and 0 <= gy < display.shape[0]:
                    display[gy, gx] = 0.25
        else:
            for gx, gy in sampled:
                if 0 <= gx < display.shape[1] and 0 <= gy < display.shape[0]:
                    display[gy, gx] = 0.25
            path_idx = i - len(sampled)
            if path_idx < len(self.path):
                for j in range(path_idx + 1):
                    gx, gy = self.path[j]
                    if 0 <= gx < display.shape[1] and 0 <= gy < display.shape[0]:
                        display[gy, gx] = 0.5

        axes.clear()
        colors = [
            [1.0, 1.0, 1.0],  # free (white)
            [0.4, 0.8, 1.0],  # explored (light blue)
            [0.0, 1.0, 0.0],  # path (green)
            [0.5, 0.5, 0.5],  # clearance (grey)
            [0.0, 0.0, 0.0],  # obstacle (black)
        ]
        cmap = ListedColormap(colors)
        axes.imshow(display,
                    extent=[self.x_range[0], self.x_range[-1],
                            self.y_range[0], self.y_range[-1]],
                    origin='lower', cmap=cmap, alpha=0.8)
        axes.plot(self.start_world[0], self.start_world[1], 'go', label="Start")
        axes.plot(self.goal_world[0], self.goal_world[1], 'ro', label="Goal")
        axes.legend()


if __name__ == "__main__":
    map_file = "map.json"
    path_file = "path.json"
    gif_path = "dstar_search.gif"

    x_lim, y_lim = MinMax(-5, 55), MinMax(-20, 25)
    start = (0, 0)
    goal = (50, -10)

    planner = DStarPathPlanner(start, goal, map_file,
                               x_lim=x_lim, y_lim=y_lim,
                               path_filename=path_file, gif_name=gif_path)

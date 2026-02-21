"""
prm_path_planner.py

Probabilistic Road Map (PRM) path planner.

Builds a graph of collision-free random samples in the configuration space,
connects neighbours within a given radius via straight-line local paths,
then uses Dijkstra to find the shortest path from start to goal through
the road map.

Reference:
    Kavraki, Svestka, Latombe & Overmars,
    "Probabilistic Roadmaps for Path Planning in High-Dimensional
    Configuration Spaces", IEEE T-RA, 1996.

Author: Erwin Lejeune
"""

import numpy as np
import heapq
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


class PrmPathPlanner:
    """
    Probabilistic Road Map path planner on a 2-D occupancy grid.

    1. Uniformly sample *n_samples* collision-free points in world space.
    2. For each sample, connect to all neighbours within *connect_radius*
       via collision-free straight lines (the "road map").
    3. Add start and goal to the road map.
    4. Run Dijkstra on the road map to find the shortest path.
    5. Optionally save a search animation GIF.

    Constructor parameters follow the project convention used by
    ``AStarPathPlanner``, ``RrtPathPlanner``, etc.
    """

    def __init__(self, start, goal, map_file, *,
                 x_lim=None, y_lim=None,
                 path_filename=None, gif_name=None,
                 n_samples=500, connect_radius=5.0,
                 line_check_samples=20, seed=42):
        self.start = start
        self.goal = goal
        self.n_samples = n_samples
        self.connect_radius = connect_radius
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

        # Deterministic sampling for reproducibility
        self._rng = np.random.default_rng(seed)

        # Build roadmap and search
        self.samples = []       # list of (x, y) world coords
        self.edges = []         # list of ((x1,y1),(x2,y2)) pairs
        self.path = []          # final path as world-coord tuples
        self._adj = {}          # adjacency list: idx -> [(idx, dist), ...]
        self._build_history = []  # snapshots for animation

        self._build_roadmap()
        self._search_dijkstra()

        # Save sparse path
        if path_filename and self.path:
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

    def _is_free(self, world_point):
        """Check if a world-coordinate point falls on a free cell."""
        gx, gy = self._world_to_grid(world_point)
        return (0 <= gx < self.cols and 0 <= gy < self.rows
                and self.grid[gy, gx] == 0)

    def _line_collision_free(self, p1, p2):
        """Check if the straight line from p1 to p2 is collision-free."""
        for i in range(self.line_check_samples + 1):
            t = i / self.line_check_samples
            x = p1[0] + t * (p2[0] - p1[0])
            y = p1[1] + t * (p2[1] - p1[1])
            if not self._is_free((x, y)):
                return False
        return True

    # -- Roadmap construction ----------------------------------------------

    def _build_roadmap(self):
        """Sample free points, connect neighbours, add start/goal."""
        # Phase 1: Sample collision-free points
        sampled = []
        attempts = 0
        max_attempts = self.n_samples * 20
        while len(sampled) < self.n_samples and attempts < max_attempts:
            x = self._rng.uniform(self.x_min, self.x_max)
            y = self._rng.uniform(self.y_min, self.y_max)
            if self._is_free((x, y)):
                sampled.append((x, y))
                # Record snapshot every 10 samples for animation
                if len(sampled) % 10 == 0:
                    self._build_history.append(
                        ('sample', len(sampled), list(sampled))
                    )
            attempts += 1

        print(f"PRM: sampled {len(sampled)} free points "
              f"({attempts} attempts)")

        # Add start and goal as nodes 0 and 1
        self.samples = [self.start, self.goal] + sampled
        n = len(self.samples)
        self._adj = {i: [] for i in range(n)}

        # Phase 2: Connect neighbours within radius
        coords = np.array(self.samples)
        edges_added = 0
        for i in range(n):
            dists = np.linalg.norm(coords - coords[i], axis=1)
            neighbours = np.where(
                (dists > 0) & (dists <= self.connect_radius)
            )[0]
            for j in neighbours:
                if j <= i:
                    continue  # avoid duplicate checks
                if self._line_collision_free(self.samples[i],
                                             self.samples[j]):
                    d = float(dists[j])
                    self._adj[i].append((j, d))
                    self._adj[j].append((i, d))
                    self.edges.append((self.samples[i], self.samples[j]))
                    edges_added += 1
                    # Record snapshot every 50 edges for animation
                    if edges_added % 50 == 0:
                        self._build_history.append(
                            ('edge', edges_added, list(self.edges))
                        )

        # Final snapshot
        self._build_history.append(
            ('edge', edges_added, list(self.edges))
        )
        print(f"PRM: {edges_added} edges in roadmap")

    # -- Dijkstra search ---------------------------------------------------

    def _search_dijkstra(self):
        """Find shortest path from start (idx 0) to goal (idx 1)."""
        start_idx, goal_idx = 0, 1
        dist = {start_idx: 0.0}
        came_from = {}
        pq = [(0.0, start_idx)]
        visited = set()
        self._dijkstra_visited = []  # for animation

        while pq:
            d, u = heapq.heappop(pq)
            if u in visited:
                continue
            visited.add(u)
            self._dijkstra_visited.append(u)

            if u == goal_idx:
                # Reconstruct path
                path = []
                cur = goal_idx
                while cur != start_idx:
                    path.append(self.samples[cur])
                    cur = came_from[cur]
                path.append(self.samples[start_idx])
                self.path = path[::-1]
                print(f"PRM: path found with {len(self.path)} waypoints")
                return

            for v, w in self._adj[u]:
                if v in visited:
                    continue
                nd = d + w
                if v not in dist or nd < dist[v]:
                    dist[v] = nd
                    came_from[v] = u
                    heapq.heappush(pq, (nd, v))

        print("PRM: no path found from start to goal")

    # -- Path utilities ----------------------------------------------------

    def _make_sparse_path(self, path, num_points=20):
        if len(path) <= num_points:
            return list(path)
        indices = np.linspace(0, len(path) - 1, num_points, dtype=int)
        return [path[i] for i in indices]

    def _save_path(self, path, filename):
        Path(filename).parent.mkdir(parents=True, exist_ok=True)
        with open(filename, 'w') as f:
            json.dump([list(p) for p in path], f)

    # -- Visualisation -----------------------------------------------------

    def visualize_search(self, gif_name=None):
        """
        Render a GIF showing:
          Phase 0: Sampling free nodes (progressive)
          Phase 1: Connecting edges (progressive)
          Phase 2: Dijkstra search expanding
          Phase 3: Path drawing
          Phase 4: Hold final
        """
        if gif_name is None:
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

        # Build frame index plan
        # Phase 0: sampling snapshots
        sample_snaps = [h for h in self._build_history if h[0] == 'sample']
        # Phase 1: edge snapshots
        edge_snaps = [h for h in self._build_history if h[0] == 'edge']
        # Phase 2: Dijkstra visited nodes
        dv = self._dijkstra_visited
        max_dijk = 40
        step_d = max(1, len(dv) // max_dijk)
        dijk_frames = list(range(0, len(dv), step_d))
        if dijk_frames and dijk_frames[-1] != len(dv) - 1:
            dijk_frames.append(len(dv) - 1)
        if not dijk_frames:
            dijk_frames = [0]
        # Phase 3: path drawing
        path_frames = 20
        # Phase 4: hold
        hold_frames = 15

        phase_lens = [
            len(sample_snaps),
            len(edge_snaps),
            len(dijk_frames),
            path_frames,
            hold_frames,
        ]
        offsets = np.cumsum([0] + phase_lens)
        total = int(offsets[-1])

        def _phase(i):
            for p in range(5):
                if i < offsets[p + 1]:
                    return p, i - int(offsets[p])
            return 4, hold_frames - 1

        titles = [
            lambda l: f"PRM — Sampling Nodes ({sample_snaps[min(l, len(sample_snaps)-1)][1]}/{self.n_samples})",
            lambda l: f"PRM — Connecting Edges ({edge_snaps[min(l, len(edge_snaps)-1)][1]}/{len(self.edges)})",
            lambda l: f"PRM — Dijkstra Search ({dijk_frames[min(l, len(dijk_frames)-1)]+1}/{len(dv)})",
            lambda _: "PRM — Shortest Path Found",
            lambda _: "PRM — Shortest Path Found",
        ]

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
                # Show sampled nodes so far
                snap = sample_snaps[min(local, len(sample_snaps) - 1)]
                pts = snap[2]
                if pts:
                    sx = [p[0] for p in pts]
                    sy = [p[1] for p in pts]
                    ax.scatter(sx, sy, c='lightblue', s=5, alpha=0.6,
                               zorder=3)

            elif phase == 1:
                # Show all samples + edges so far
                all_pts = self.samples[2:]  # exclude start/goal
                if all_pts:
                    ax.scatter([p[0] for p in all_pts],
                               [p[1] for p in all_pts],
                               c='lightblue', s=5, alpha=0.4, zorder=3)
                snap = edge_snaps[min(local, len(edge_snaps) - 1)]
                edges_so_far = snap[2]
                for e in edges_so_far:
                    ax.plot([e[0][0], e[1][0]], [e[0][1], e[1][1]],
                            'b-', linewidth=0.3, alpha=0.3, zorder=2)

            elif phase >= 2:
                # Always show full roadmap in background
                all_pts = self.samples[2:]
                if all_pts:
                    ax.scatter([p[0] for p in all_pts],
                               [p[1] for p in all_pts],
                               c='lightblue', s=5, alpha=0.3, zorder=3)
                for e in self.edges:
                    ax.plot([e[0][0], e[1][0]], [e[0][1], e[1][1]],
                            'b-', linewidth=0.3, alpha=0.15, zorder=2)

                if phase == 2:
                    # Highlight visited nodes
                    idx = dijk_frames[min(local, len(dijk_frames) - 1)]
                    visited_pts = [self.samples[v]
                                   for v in dv[:idx + 1]]
                    if visited_pts:
                        ax.scatter([p[0] for p in visited_pts],
                                   [p[1] for p in visited_pts],
                                   c='orange', s=15, alpha=0.7,
                                   zorder=4)

                elif phase >= 3:
                    # Draw path progressively or fully
                    if self.path:
                        if phase == 3:
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
                                    linewidth=2.5, zorder=5,
                                    label="Path")

            # Always draw start and goal
            ax.plot(self.start[0], self.start[1], 'go', markersize=10,
                    label="Start", zorder=6)
            ax.plot(self.goal[0], self.goal[1], 'ro', markersize=10,
                    label="Goal", zorder=6)

            ax.set_title(titles[phase](local), fontsize=14)
            ax.legend(loc='upper left')
            ax.set_xlabel("X [m]", fontsize=12)
            ax.set_ylabel("Y [m]", fontsize=12)
            ax.set_aspect("equal")

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111)
        ax.set_aspect("equal")

        print(f"PRM search animation: {total} frames")
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
    gif_path = "prm_search.gif"

    x_lim, y_lim = MinMax(-5, 55), MinMax(-20, 25)
    start = (0, 0)
    goal = (50, -10)

    planner = PrmPathPlanner(
        start, goal, map_file,
        x_lim=x_lim, y_lim=y_lim,
        path_filename=path_file,
        gif_name=gif_path,
        n_samples=500,
        connect_radius=5.0,
    )

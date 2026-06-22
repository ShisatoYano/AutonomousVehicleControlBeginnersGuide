"""
q_learning_path_planner.py

Q-learning path planner.

The planner learns action values on a 2-D occupancy grid. Episodes start
at the start cell, use epsilon-greedy exploration, and update a Q-table
with rewards for reaching the goal and penalties for long or repeated
motion. The learned greedy policy is converted to a path for navigation.

Constructor parameters follow the project convention used by
``AStarPathPlanner``, ``PrmPathPlanner``, ``PsoPathPlanner``, etc.

Author: Banaan Kiamanesh
GitHub: https://github.com/BanaanKiamanesh
"""

from collections import deque
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


class QLearningPathPlanner:
    """Q-learning path planner on a 2-D occupancy grid."""

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
        n_episodes=450,
        alpha=0.15,
        gamma=0.98,
        epsilon_start=1.0,
        epsilon_min=0.05,
        epsilon_decay=0.990,
        goal_reward=120.0,
        step_penalty=1.0,
        revisit_penalty=0.25,
        diagonal_motion=True,
        max_steps=None,
        bootstrap_path=True,
        seed=10,
    ):
        self.start = tuple(start)
        self.goal = tuple(goal)
        self.n_episodes = n_episodes
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon_start = epsilon_start
        self.epsilon_min = epsilon_min
        self.epsilon_decay = epsilon_decay
        self.goal_reward = goal_reward
        self.step_penalty = step_penalty
        self.revisit_penalty = revisit_penalty
        self.diagonal_motion = diagonal_motion
        self.bootstrap_path = bootstrap_path
        self.rng = np.random.default_rng(seed)

        self.raw_grid = self._load_grid(map_file)
        if x_lim is None or y_lim is None:
            raise ValueError("x_lim and y_lim are required for world/grid conversion.")

        x_min, x_max = x_lim.min_value(), x_lim.max_value()
        y_min, y_max = y_lim.min_value(), y_lim.max_value()
        self.resolution = (x_max - x_min) / self.raw_grid.shape[1]
        self.x_range = np.arange(x_min, x_max, self.resolution)
        self.y_range = np.arange(y_min, y_max, self.resolution)

        # BinaryOccupancyGrid stores free cells as 0.0, clearance as 0.75,
        # and obstacles as 1.0. Path planners in this repo avoid both
        # clearance and obstacle cells.
        self.grid = (self.raw_grid > 0).astype(np.uint8)
        self.rows, self.cols = self.grid.shape
        self.n_states = self.rows * self.cols
        self.max_steps = max_steps or 4 * (self.rows + self.cols)

        self.start_idx = self._world_to_grid_rc(self.start)
        self.goal_idx = self._world_to_grid_rc(self.goal)
        self._validate_endpoint(self.start_idx, "start")
        self._validate_endpoint(self.goal_idx, "goal")
        self.start_state = self.node_id(self.start_idx)
        self.goal_state = self.node_id(self.goal_idx)

        self.actions = self.build_actions()
        self.n_actions = len(self.actions)
        self.q_table = np.zeros((self.n_states, self.n_actions), dtype=float)
        self.valid_actions = self.precompute_valid_actions()
        self.goal_distance = self.precompute_goal_distance()

        self.bfs_path = self.shortest_path_bfs()
        if self.bfs_path is None:
            raise ValueError("No feasible path exists between start and goal.")
        if self.bootstrap_path:
            self._bootstrap_q_table_from_path(self.bfs_path)

        self.episode_rewards = []
        self.episode_steps = []
        self.best_path = None
        self.best_cost = np.inf
        self.best_cost_history = []
        self.best_path_history = []
        self.grid_path = []
        self.path = []

        self.train()

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

    def _world_to_grid_rc(self, point):
        col = int((point[0] - self.x_range[0]) / self.resolution)
        row = int((point[1] - self.y_range[0]) / self.resolution)
        return row, col

    def _grid_to_world(self, node):
        row, col = node
        return (
            self.x_range[0] + col * self.resolution,
            self.y_range[0] + row * self.resolution,
        )

    def _in_bounds(self, node):
        return 0 <= node[0] < self.rows and 0 <= node[1] < self.cols

    def _validate_endpoint(self, node, name):
        if not self._in_bounds(node):
            raise ValueError(f"{name.capitalize()} point is outside the map.")
        if self.grid[node] == 1:
            raise ValueError(f"{name.capitalize()} point is inside an obstacle.")

    def build_actions(self):
        if self.diagonal_motion:
            return [
                (-1, 0, 1.0),
                (1, 0, 1.0),
                (0, -1, 1.0),
                (0, 1, 1.0),
                (-1, -1, np.sqrt(2.0)),
                (-1, 1, np.sqrt(2.0)),
                (1, -1, np.sqrt(2.0)),
                (1, 1, np.sqrt(2.0)),
            ]

        return [
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
        ]

    def node_id(self, node):
        return node[0] * self.cols + node[1]

    def node_rc(self, state):
        return divmod(state, self.cols)

    def precompute_valid_actions(self):
        valid_actions = [[] for _ in range(self.n_states)]

        for r in range(self.rows):
            for c in range(self.cols):
                if self.grid[r, c] == 1:
                    continue

                state = self.node_id((r, c))

                for action_id, (dr, dc, _) in enumerate(self.actions):
                    nr = r + dr
                    nc = c + dc

                    if not (0 <= nr < self.rows and 0 <= nc < self.cols):
                        continue

                    if self.grid[nr, nc] == 1:
                        continue

                    if abs(dr) == 1 and abs(dc) == 1:
                        if self.grid[r + dr, c] == 1 or self.grid[r, c + dc] == 1:
                            continue

                    valid_actions[state].append(action_id)

        return [np.array(actions, dtype=np.int32) for actions in valid_actions]

    def precompute_goal_distance(self):
        goal_r, goal_c = self.goal_idx
        distance = np.empty(self.n_states, dtype=float)

        for state in range(self.n_states):
            r, c = self.node_rc(state)
            distance[state] = np.hypot(r - goal_r, c - goal_c)

        return distance

    def shortest_path_bfs(self):
        queue = deque([self.start_state])
        previous = np.full(self.n_states, -1, dtype=np.int32)
        visited = np.zeros(self.n_states, dtype=bool)
        visited[self.start_state] = True

        while queue:
            current = queue.popleft()
            if current == self.goal_state:
                break

            for action_id in self.valid_actions[current]:
                next_state, _, _ = self.step(current, action_id, set())
                if not visited[next_state]:
                    visited[next_state] = True
                    previous[next_state] = current
                    queue.append(next_state)

        if not visited[self.goal_state]:
            return None

        path = []
        current = self.goal_state
        while current != -1:
            path.append(current)
            current = previous[current]

        path.reverse()
        return self.states_to_path(path)

    def _action_between(self, current, next_node):
        dr = next_node[0] - current[0]
        dc = next_node[1] - current[1]
        for action_id, (action_dr, action_dc, _) in enumerate(self.actions):
            if dr == action_dr and dc == action_dc:
                return action_id
        return None

    def _bootstrap_q_table_from_path(self, path):
        """Seed the Q-table with a known feasible route for the large demo map."""
        n = len(path)
        for index, (current, next_node) in enumerate(zip(path[:-1], path[1:])):
            action_id = self._action_between(current, next_node)
            if action_id is None:
                continue
            state = self.node_id(current)
            remaining = n - index
            self.q_table[state, action_id] = self.goal_reward / max(1, remaining)

    def select_action(self, state, epsilon):
        actions = self.valid_actions[state]

        if actions.size == 0:
            return None

        if self.rng.random() < epsilon:
            return int(self.rng.choice(actions))

        action_values = self.q_table[state, actions]
        max_value = np.max(action_values)
        best_actions = actions[action_values == max_value]

        return int(self.rng.choice(best_actions))

    def step(self, state, action_id, visited):
        r, c = self.node_rc(state)
        dr, dc, move_cost = self.actions[action_id]
        nr = r + dr
        nc = c + dc
        next_state = self.node_id((nr, nc))

        reward = -self.step_penalty * move_cost
        reward += 0.05 * (self.goal_distance[state] - self.goal_distance[next_state])
        done = False

        if next_state in visited:
            reward -= self.revisit_penalty

        if next_state == self.goal_state:
            reward += self.goal_reward
            done = True

        return next_state, reward, done

    def train(self):
        epsilon = self.epsilon_start

        for _ in range(self.n_episodes):
            state = self.start_state
            total_reward = 0.0
            visited = {state}
            path_states = [state]
            reached_goal = False
            step_number = 0

            for step_number in range(1, self.max_steps + 1):
                action_id = self.select_action(state, epsilon)
                if action_id is None:
                    break

                next_state, reward, done = self.step(state, action_id, visited)
                next_actions = self.valid_actions[next_state]

                if next_actions.size > 0:
                    next_best_q = np.max(self.q_table[next_state, next_actions])
                else:
                    next_best_q = 0.0

                old_q = self.q_table[state, action_id]
                target = reward + self.gamma * next_best_q * (not done)
                self.q_table[state, action_id] = old_q + self.alpha * (target - old_q)

                total_reward += reward
                state = next_state
                visited.add(state)
                path_states.append(state)

                if done:
                    reached_goal = True
                    break

            if reached_goal:
                clean_states = self._remove_cycles(path_states)
                candidate_path = self.states_to_path(clean_states)
                candidate_path = self._densify_path(
                    self._shortcut_path(self._prune_collinear(candidate_path))
                )
                cost = self.path_cost(candidate_path)
                if cost < self.best_cost:
                    self.best_cost = cost
                    self.best_path = candidate_path

            self.episode_rewards.append(total_reward)
            self.episode_steps.append(step_number)
            self.best_cost_history.append(self.best_cost)
            self.best_path_history.append(list(self.best_path) if self.best_path else None)

            epsilon = max(self.epsilon_min, epsilon * self.epsilon_decay)

        greedy_path = self.extract_greedy_path()
        if greedy_path is not None:
            greedy_path = self._densify_path(
                self._shortcut_path(self._prune_collinear(greedy_path))
            )
            greedy_cost = self.path_cost(greedy_path)
            if greedy_cost < self.best_cost:
                self.best_path = greedy_path
                self.best_cost = greedy_cost

        if self.best_path is None:
            self.best_path = self._densify_path(
                self._shortcut_path(self._prune_collinear(self.bfs_path))
            )
            self.best_cost = self.path_cost(self.best_path)
            print("Q-learning: using BFS fallback path after training.")

        self.grid_path = self.best_path
        self.path = [self._grid_to_world(node) for node in self.grid_path]
        print(
            f"Q-learning: best path cost={self.best_cost:.2f}, "
            f"grid_waypoints={len(self.grid_path)}"
        )

        return self.best_path, self.best_cost

    def extract_greedy_path(self):
        state = self.start_state
        path_states = [state]
        visited = {state}

        for _ in range(self.max_steps):
            if state == self.goal_state:
                return self.states_to_path(path_states)

            actions = self.valid_actions[state]
            if actions.size == 0:
                return None

            action_values = self.q_table[state, actions]
            sorted_indices = np.argsort(action_values)[::-1]
            selected_next_state = None

            for index in sorted_indices:
                action_id = int(actions[index])
                next_state, _, _ = self.step(state, action_id, visited)

                if next_state not in visited or next_state == self.goal_state:
                    selected_next_state = next_state
                    break

            if selected_next_state is None:
                return None

            state = selected_next_state
            path_states.append(state)
            visited.add(state)

        if path_states[-1] == self.goal_state:
            return self.states_to_path(path_states)
        return None

    @staticmethod
    def _remove_cycles(states):
        cleaned = []
        index_by_state = {}
        for state in states:
            if state in index_by_state:
                keep_until = index_by_state[state] + 1
                for removed in cleaned[keep_until:]:
                    index_by_state.pop(removed, None)
                cleaned = cleaned[:keep_until]
                continue
            index_by_state[state] = len(cleaned)
            cleaned.append(state)
        return cleaned

    def states_to_path(self, states):
        return [self.node_rc(state) for state in states]

    def path_cost_states(self, states):
        cost = 0.0

        for i in range(len(states) - 1):
            r1, c1 = self.node_rc(states[i])
            r2, c2 = self.node_rc(states[i + 1])
            cost += np.hypot(r2 - r1, c2 - c1)

        return cost

    @staticmethod
    def path_cost(path):
        cost = 0.0

        for i in range(len(path) - 1):
            r1, c1 = path[i]
            r2, c2 = path[i + 1]
            cost += np.hypot(r2 - r1, c2 - c1)

        return cost

    def _line_is_free(self, start, goal):
        dr = goal[0] - start[0]
        dc = goal[1] - start[1]
        steps = max(abs(dr), abs(dc)) * 2
        if steps == 0:
            return self.grid[start] == 0

        for i in range(steps + 1):
            t = i / steps
            node = (
                int(round(start[0] + dr * t)),
                int(round(start[1] + dc * t)),
            )
            if not self._in_bounds(node) or self.grid[node] == 1:
                return False
        return True

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
            dr = goal[0] - start[0]
            dc = goal[1] - start[1]
            steps = max(1, int(np.ceil(max(abs(dr), abs(dc)) / max_step_cells)))
            for step in range(1, steps + 1):
                node = (
                    int(round(start[0] + dr * step / steps)),
                    int(round(start[1] + dc * step / steps)),
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

    def get_value_map(self):
        value_map = np.full(self.grid.shape, np.nan, dtype=float)

        for state in range(self.n_states):
            r, c = self.node_rc(state)

            if self.grid[r, c] == 1:
                continue

            actions = self.valid_actions[state]
            if actions.size == 0:
                value_map[r, c] = 0.0
            else:
                value_map[r, c] = np.max(self.q_table[state, actions])

        return value_map

    def get_policy_arrows(self, stride=6, min_abs_q=1e-6):
        x = []
        y = []
        u = []
        v = []

        for r in range(0, self.rows, stride):
            for c in range(0, self.cols, stride):
                state = self.node_id((r, c))

                if self.grid[r, c] == 1 or state == self.goal_state:
                    continue

                actions = self.valid_actions[state]
                if actions.size == 0:
                    continue

                action_values = self.q_table[state, actions]
                if np.max(np.abs(action_values)) <= min_abs_q:
                    continue

                best_action = actions[np.argmax(action_values)]
                dr, dc, _ = self.actions[best_action]

                wx, wy = self._grid_to_world((r, c))
                x.append(wx)
                y.append(wy)
                u.append(dc * self.resolution)
                v.append(dr * self.resolution)

        return np.array(x), np.array(y), np.array(u), np.array(v)

    def visualize_search(self, gif_name=None):
        """Render a GIF showing training progress and the learned policy."""
        if gif_name is None:
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
        max_search_frames = 60
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
                candidate = self.best_path_history[idx]
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
                    f"Q-learning Training ({idx + 1}/{self.n_episodes}) | cost={cost_text}",
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

                if phase == 2:
                    x, y, u, v = self.get_policy_arrows()
                    ax.quiver(
                        x,
                        y,
                        u,
                        v,
                        angles="xy",
                        scale_units="xy",
                        scale=1.8,
                        width=0.003,
                        color="#455A64",
                        alpha=0.45,
                    )
                ax.set_title("Q-learning Final Policy", fontsize=14)

            ax.plot(self.start[0], self.start[1], "go", markersize=10, label="Start")
            ax.plot(self.goal[0], self.goal[1], "ro", markersize=10, label="Goal")
            ax.legend(loc="upper left")
            ax.set_xlabel("X [m]", fontsize=12)
            ax.set_ylabel("Y [m]", fontsize=12)
            ax.set_aspect("equal")

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111)
        ax.set_aspect("equal")

        print(f"Q-learning search animation: {total} frames")
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
    gif_path = "q_learning_search.gif"

    x_lim, y_lim = MinMax(-5, 55), MinMax(-20, 25)
    start = (0, 0)
    goal = (50, -10)

    planner = QLearningPathPlanner(
        start,
        goal,
        map_file,
        x_lim=x_lim,
        y_lim=y_lim,
        path_filename=path_file,
        gif_name=gif_path,
    )

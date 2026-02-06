"""
gradient_path_planner.py

Author: Panav Arpit Raaj

Gradient descent path planner on pre-computed potential fields.
Follows the negative gradient from start to goal.
"""

import numpy as np
import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class GradientPathPlanner:
    """
    Loads a potential field from JSON and follows steepest descent
    from start to goal. Detects local minima when gradient vanishes.
    """

    def __init__(self, potential_file, start, step_size=0.3,
                 max_iterations=10000, goal_tolerance=1.0,
                 stuck_threshold=1e-5, stuck_count_limit=20):
        """
        potential_file: JSON with potential grid + metadata
        start: (x, y) world coordinates
        step_size: metres per iteration
        goal_tolerance: close-enough distance to goal [m]
        stuck_threshold: gradient magnitude treated as zero
        stuck_count_limit: consecutive zero-gradient steps before giving up
        """
        self.start = start
        self.step_size = step_size
        self.max_iterations = max_iterations
        self.goal_tolerance = goal_tolerance
        self.stuck_threshold = stuck_threshold
        self.stuck_count_limit = stuck_count_limit

        # Load the potential field from JSON
        self._load_potential_field(potential_file)

    def _load_potential_field(self, filepath):
        """Parse JSON exported by PotentialFieldMap.export_to_json()."""
        with open(filepath, 'r') as f:
            data = json.load(f)

        self.resolution = data['metadata']['resolution']
        self.width = data['metadata']['width']
        self.height = data['metadata']['height']
        self.origin = data['metadata']['origin']
        self.goal = tuple(data['goal_pos'])
        self.potential_grid = np.array(data['potential_grid'])  # (height, width)

    def _world_to_grid(self, x, y):
        """World (x,y) metres -> grid (col, row) indices. None if out of bounds."""
        col_idx = int((x - self.origin[0]) / self.resolution)
        row_idx = int((y - self.origin[1]) / self.resolution)

        if 0 <= col_idx < self.width and 0 <= row_idx < self.height:
            return (col_idx, row_idx)
        return None

    def _get_potential(self, x, y):
        """Potential value at world (x,y). Returns None if out of bounds."""
        grid_pos = self._world_to_grid(x, y)
        if grid_pos is None:
            return None
        col_idx, row_idx = grid_pos
        return self.potential_grid[row_idx, col_idx]

    # --- Gradient via central finite differences ---
    # dU/dx ≈ (U(x+h,y) - U(x-h,y)) / 2h
    # dU/dy ≈ (U(x,y+h) - U(x,y-h)) / 2h

    def _compute_gradient(self, x, y):
        """Gradient at (x,y). Falls back to one-sided differences at edges."""
        h = self.resolution

        U_xp = self._get_potential(x + h, y)
        U_xn = self._get_potential(x - h, y)
        U_yp = self._get_potential(x, y + h)
        U_yn = self._get_potential(x, y - h)

        if U_xp is None or U_xn is None or U_yp is None or U_yn is None:
            return self._compute_boundary_gradient(x, y, h)

        grad_x = (U_xp - U_xn) / (2 * h)
        grad_y = (U_yp - U_yn) / (2 * h)
        return (grad_x, grad_y)

    def _compute_boundary_gradient(self, x, y, h):
        """One-sided finite differences when neighbours are out of bounds."""
        U_center = self._get_potential(x, y)
        if U_center is None:
            return (0.0, 0.0)

        grad_x = 0.0
        grad_y = 0.0

        U_xp = self._get_potential(x + h, y)
        U_xn = self._get_potential(x - h, y)
        if U_xp is not None and U_xn is not None:
            grad_x = (U_xp - U_xn) / (2 * h)
        elif U_xp is not None:
            grad_x = (U_xp - U_center) / h
        elif U_xn is not None:
            grad_x = (U_center - U_xn) / h

        U_yp = self._get_potential(x, y + h)
        U_yn = self._get_potential(x, y - h)
        if U_yp is not None and U_yn is not None:
            grad_y = (U_yp - U_yn) / (2 * h)
        elif U_yp is not None:
            grad_y = (U_yp - U_center) / h
        elif U_yn is not None:
            grad_y = (U_center - U_yn) / h

        return (grad_x, grad_y)

    def plan(self):
        """
        Follow negative gradient from start until goal, local minimum, or max iterations.
        Returns dict with path, success flag, and diagnostics.
        """
        path = [self.start]
        current = np.array(self.start, dtype=float)
        stuck_count = 0

        for iteration in range(self.max_iterations):
            dist_to_goal = np.linalg.norm(current - np.array(self.goal))
            if dist_to_goal < self.goal_tolerance:
                return self._create_result(
                    path, True, "goal_reached", iteration, False, dist_to_goal
                )

            grad = self._compute_gradient(current[0], current[1])
            grad_magnitude = np.sqrt(grad[0]**2 + grad[1]**2)

            # Local minimum: gradient ≈ 0 but not at goal
            if grad_magnitude < self.stuck_threshold:
                stuck_count += 1
                if stuck_count >= self.stuck_count_limit:
                    print(f"[WARNING] Local minimum at "
                          f"({current[0]:.2f}, {current[1]:.2f}), "
                          f"dist to goal: {dist_to_goal:.2f}m")
                    return self._create_result(
                        path, False, "local_minimum", iteration, True, dist_to_goal
                    )
            else:
                stuck_count = 0

            # Step along -∇U (downhill)
            if grad_magnitude > 0:
                direction = np.array([-grad[0], -grad[1]]) / grad_magnitude
                current = current + direction * self.step_size
                path.append(tuple(current))

        dist_to_goal = np.linalg.norm(current - np.array(self.goal))
        return self._create_result(
            path, False, "max_iterations", self.max_iterations, False, dist_to_goal
        )

    def _create_result(self, path, success, reason, iterations, local_min, dist):
        """Pack planning outcome into a dict and log it."""
        result = {
            'path': path,
            'success': success,
            'termination_reason': reason,
            'iterations': iterations,
            'local_minimum_detected': local_min,
            'final_distance_to_goal': dist
        }
        if success:
            print(f"[SUCCESS] Goal reached in {iterations} iterations")
        elif local_min:
            print(f"[LOCAL MINIMUM] Stuck at iteration {iterations}")
        else:
            print(f"[TIMEOUT] {iterations} iterations exhausted")
        return result

    def save_path(self, path, filepath):
        """Write path as JSON list of (x,y) pairs."""
        with open(filepath, 'w') as f:
            json.dump(path, f)

    def visualize(self, result, output_gif=None, show_plot=True):
        """Overlay planned path on the potential field heatmap."""
        fig, ax = plt.subplots(figsize=(12, 9))
        ax.set_aspect('equal')
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_title('Gradient Descent on Potential Field')

        # Cell edges for pcolormesh
        x_edges = np.linspace(
            self.origin[0],
            self.origin[0] + self.width * self.resolution,
            self.width + 1)
        y_edges = np.linspace(
            self.origin[1],
            self.origin[1] + self.height * self.resolution,
            self.height + 1)

        pcm = ax.pcolormesh(x_edges, y_edges, self.potential_grid,
                            cmap='jet', alpha=0.6, shading='flat')
        fig.colorbar(pcm, ax=ax, label='Potential')

        ax.plot(*self.start, 'go', markersize=12, label='Start', zorder=5)
        ax.plot(*self.goal, 'r*', markersize=15, label='Goal', zorder=5)

        path = result['path']

        if output_gif and len(path) > 1:
            line, = ax.plot([], [], 'w-', linewidth=2, label='Path')
            point, = ax.plot([], [], 'wo', markersize=8,
                             markeredgecolor='black')

            def init():
                line.set_data([], [])
                point.set_data([], [])
                return line, point

            def animate(frame_idx):
                step = max(1, len(path) // 200)
                i = min(frame_idx * step, len(path) - 1)
                xs = [p[0] for p in path[:i+1]]
                ys = [p[1] for p in path[:i+1]]
                line.set_data(xs, ys)
                point.set_data([path[i][0]], [path[i][1]])
                return line, point

            anim = animation.FuncAnimation(
                fig, animate, init_func=init,
                frames=min(len(path), 200), interval=50, blit=True)
            anim.save(output_gif, writer='pillow')
            print(f"GIF saved: {output_gif}")
        else:
            if len(path) > 1:
                path_x = [p[0] for p in path]
                path_y = [p[1] for p in path]
                ax.plot(path_x, path_y, 'w-', linewidth=2, label='Path')

        if result['local_minimum_detected']:
            ax.plot(*path[-1], 'yx', markersize=15,
                    markeredgewidth=3, label='Local Minimum', zorder=6)

        ax.legend(loc='upper right')
        if show_plot:
            plt.show()
        plt.close()

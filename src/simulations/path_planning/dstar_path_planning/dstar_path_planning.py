"""
dstar_path_planning.py

Simulation that demonstrates D*'s incremental replanning.

Two GIFs are produced:
    1. **dstar_search.gif** – grid-based animation showing D* expansion,
       initial path, dynamic obstacle injection, replan expansion, and
       the replanned path.
    2. **dstar_navigate.gif** – car-following navigation using the same
       vehicle / pure-pursuit stack as the other planners.  Midway through,
       a new obstacle appears on the initial path, D* replans, and the car
       seamlessly switches to following the new route.
"""

import numpy as np
import sys
import json
import matplotlib.pyplot as plt
import matplotlib.animation as anm
from pathlib import Path
from matplotlib.colors import ListedColormap

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"
relative_simulations = "/../../../simulations/"

sys.path.append(abs_dir_path + relative_path + "visualization")
sys.path.append(abs_dir_path + relative_path + "state")
sys.path.append(abs_dir_path + relative_path + "vehicle")
sys.path.append(abs_dir_path + relative_path + "obstacle")
sys.path.append(abs_dir_path + relative_path + "mapping/grid")
sys.path.append(abs_dir_path + relative_path + "course/cubic_spline_course")
sys.path.append(abs_dir_path + relative_path + "control/pure_pursuit")
sys.path.append(abs_dir_path + relative_path + "plan/dstar")

from global_xy_visualizer import GlobalXYVisualizer
from min_max import MinMax
from time_parameters import TimeParameters
from vehicle_specification import VehicleSpecification
from state import State
from four_wheels_vehicle import FourWheelsVehicle
from obstacle import Obstacle
from obstacle_list import ObstacleList
from cubic_spline_course import CubicSplineCourse
from pure_pursuit_controller import PurePursuitController
from binary_occupancy_grid import BinaryOccupancyGrid
from dstar_path_planner import DStarPathPlanner

# flag to show plot figure
# when executed as unit test, this flag is set as false
show_plot = True


# ---- helpers ---------------------------------------------------------------

def _build_obstacle_cells(planner, cx, cy, half_w, half_h):
    """Convert a world-space rectangle into a list of (row, col) grid cells."""
    cells = []
    for wx in np.arange(cx - half_w, cx + half_w, planner.resolution):
        for wy in np.arange(cy - half_h, cy + half_h, planner.resolution):
            r = int((wy - planner.y_range[0]) / planner.resolution)
            c = int((wx - planner.x_range[0]) / planner.resolution)
            if 0 <= r < planner.rows and 0 <= c < planner.cols:
                cells.append((r, c))
    return cells


def _subsample(lst, max_frames):
    """Return at most *max_frames* evenly-spaced items from *lst*."""
    if len(lst) <= max_frames:
        return list(lst)
    step = max(1, len(lst) // max_frames)
    return list(lst[::step])


def _path_to_course(planner, grid_path, speed_kmph=20, color='r'):
    """Convert a D* grid path to a CubicSplineCourse."""
    world_pts = []
    for gx, gy in grid_path:
        wx = planner.x_range[0] + gx * planner.resolution
        wy = planner.y_range[0] + gy * planner.resolution
        world_pts.append((wx, wy))

    # Sub-sample to ~20 control points for a smooth spline
    if len(world_pts) > 20:
        indices = np.linspace(0, len(world_pts) - 1, 20, dtype=int)
        world_pts = [world_pts[i] for i in indices]

    xs = [p[0] for p in world_pts]
    ys = [p[1] for p in world_pts]
    return CubicSplineCourse(xs, ys, speed_kmph, color=color)


class _ReplanPurePursuit:
    """
    A wrapper around PurePursuitController that switches to a new course
    when the vehicle gets close to the dynamic-obstacle region.

    Manages all course visualisation so the viewer sees:
      - Before replan: initial course drawn as red dots (standard look).
      - After replan: initial course fades to dotted light-grey,
        replanned course drawn as red dots, dynamic obstacle appears.

    PurePursuitController uses default green target-point colour for both
    controllers, matching the standard simulations.
    """

    def __init__(self, spec, initial_course, replanned_course,
                 switch_x, switch_y, switch_radius,
                 dynamic_obst_list):
        self._spec = spec
        self._initial_course = initial_course
        self._replanned_course = replanned_course
        self._initial_ctrl = PurePursuitController(spec, initial_course)
        self._replan_ctrl = PurePursuitController(spec, replanned_course)
        self._active = self._initial_ctrl
        self._switched = False
        self._switch_x = switch_x
        self._switch_y = switch_y
        self._switch_radius = switch_radius
        self._dyn_obst_list = dynamic_obst_list

    def update(self, state, time_s):
        # Check if we should switch
        if not self._switched:
            dx = state.get_x_m() - self._switch_x
            dy = state.get_y_m() - self._switch_y
            if (dx * dx + dy * dy) ** 0.5 < self._switch_radius:
                self._switched = True
                self._active = self._replan_ctrl
        self._active.update(state, time_s)

    def get_target_accel_mps2(self):
        return self._active.get_target_accel_mps2()

    def get_target_steer_rad(self):
        return self._active.get_target_steer_rad()

    def get_target_yaw_rate_rps(self):
        return self._active.get_target_yaw_rate_rps()

    def draw(self, axes, elems):
        if self._switched:
            # Old course → dotted light grey line
            old, = axes.plot(self._initial_course.x_array,
                             self._initial_course.y_array,
                             linestyle=':', linewidth=1.2,
                             color='#AAAAAA', label="Old Course")
            elems.append(old)
            # New course → red dots (standard look)
            self._replanned_course.draw(axes, elems)
            # Dynamic obstacle becomes visible
            self._dyn_obst_list.draw(axes, elems)
        else:
            # Before replan: draw initial course as red dots (standard)
            self._initial_course.draw(axes, elems)
        # Active PurePursuit draws its green target point (standard)
        self._active.draw(axes, elems)


# ---- search GIF -----------------------------------------------------------

# Grid cell colour values used in the animation.
_V_FREE = 0.0       # white
_V_EXPLORED = 0.25  # light blue
_V_REPLAN_EX = 0.35 # orange-ish
_V_PATH = 0.50      # green
_V_OLD_PATH = 0.60  # light grey (faded old path)
_V_CLEARANCE = 0.75 # mid-grey
_V_OBSTACLE = 1.0   # black


def _build_search_gif(planner, initial_path, initial_explored,
                      grid_before_obstacle, grid_after_obstacle,
                      replan_explored, replanned_path,
                      start, goal, gif_path):
    """
    Render the grid-based search / replan animation.

    Six phases:
      0. Initial search expansion (light blue cells, animated).
      1. Initial path drawn progressively (green cells).
      2. Hold — admire the initial path.
      3. Obstacle appears on grid, old path turns grey — hold.
      4. D* replan expansion from start outward (orange cells, animated).
      5. Replanned path drawn progressively (green cells).
      6. Hold — admire the replanned result.
    """

    # Sub-sampled frame lists per phase
    ph_search   = _subsample(initial_explored, 80)
    ph_ipath    = _subsample(initial_path, 30)
    ph_hold1    = 20                                          # admire initial
    ph_obst     = 25                                          # obstacle + grey
    ph_replan   = _subsample(replan_explored, 60)
    ph_rpath    = _subsample(replanned_path, 30) if replanned_path else []
    ph_hold2    = 25                                          # admire final

    lengths = [len(ph_search), len(ph_ipath), ph_hold1,
               ph_obst, len(ph_replan), len(ph_rpath), ph_hold2]
    offsets = np.cumsum([0] + lengths)
    total_frames = int(offsets[-1])
    n_phases = len(lengths)

    def _phase_and_local(i):
        for p in range(n_phases):
            if i < offsets[p + 1]:
                return p, i - int(offsets[p])
        return n_phases - 1, lengths[-1] - 1

    cmap = ListedColormap([
        [1.0, 1.0, 1.0],    # 0  free        → white
        [0.4, 0.8, 1.0],    # 1  explored     → light blue
        [1.0, 0.6, 0.2],    # 2  replan exp   → orange
        [0.0, 0.8, 0.0],    # 3  path         → green
        [0.78, 0.78, 0.78], # 4  old path     → light grey
        [0.5, 0.5, 0.5],    # 5  clearance    → mid-grey
        [0.0, 0.0, 0.0],    # 6  obstacle     → black
    ])

    def _disc(display):
        """Map continuous grid values to discrete colour indices."""
        d = np.zeros_like(display, dtype=int)
        d[display == _V_FREE] = 0
        d[np.isclose(display, _V_EXPLORED)] = 1
        d[np.isclose(display, _V_REPLAN_EX)] = 2
        d[np.isclose(display, _V_PATH)] = 3
        d[np.isclose(display, _V_OLD_PATH)] = 4
        d[np.isclose(display, _V_CLEARANCE)] = 5
        d[display >= 0.99] = 6
        return d

    def _paint(display, cells, val):
        for gx, gy in cells:
            if 0 <= gx < display.shape[1] and 0 <= gy < display.shape[0]:
                if display[gy, gx] < 0.99:
                    display[gy, gx] = val

    # Phase-dependent background grid:
    #   phases 0-2 use grid_before_obstacle (no dynamic obstacle yet)
    #   phases 3+  use grid_after_obstacle  (obstacle present)
    def _base_grid(phase):
        return (grid_before_obstacle if phase <= 2
                else grid_after_obstacle).copy()

    # ---- rendering helpers that accumulate per-phase ----

    def _draw_initial_explored(display):
        """Paint all initial-search explored cells (blue)."""
        _paint(display, ph_search, _V_EXPLORED)

    def _draw_initial_path(display):
        """Paint the full initial path (green)."""
        _paint(display, initial_path, _V_PATH)

    def _draw_old_path_grey(display):
        """Paint the initial path as faded grey (superseded)."""
        _paint(display, initial_path, _V_OLD_PATH)

    titles = [
        lambda l: f"D* Initial Search ({min(l+1, len(ph_search))}/{len(ph_search)})",
        lambda _: "Initial Path Found",
        lambda _: "Initial Path Found",
        lambda _: "Obstacle Detected!",
        lambda l: f"D* Replanning from Start ({min(l+1, len(ph_replan))}/{len(ph_replan)})",
        lambda _: "Replanned Path Found",
        lambda _: "Replanned Path Found",
    ]

    def update_frame(i, axes):
        phase, local = _phase_and_local(i)
        display = _base_grid(phase)

        if phase == 0:
            # Animate initial search expansion
            _paint(display, ph_search[:local + 1], _V_EXPLORED)

        elif phase == 1:
            # Draw all explored + animate initial path
            _draw_initial_explored(display)
            _paint(display, ph_ipath[:local + 1], _V_PATH)

        elif phase == 2:
            # Hold: full explored + full initial path
            _draw_initial_explored(display)
            _draw_initial_path(display)

        elif phase == 3:
            # Obstacle has appeared (grid_after_obstacle).
            # Old explored stays, old path turns grey.
            _draw_initial_explored(display)
            _draw_old_path_grey(display)

        elif phase == 4:
            # Animate replan expansion; old path stays grey
            _draw_initial_explored(display)
            _draw_old_path_grey(display)
            _paint(display, ph_replan[:local + 1], _V_REPLAN_EX)

        elif phase == 5:
            # All replan explored + animate replanned path
            _draw_initial_explored(display)
            _draw_old_path_grey(display)
            _paint(display, ph_replan, _V_REPLAN_EX)
            _paint(display, ph_rpath[:local + 1], _V_PATH)

        else:  # phase == 6
            # Hold: final result
            _draw_initial_explored(display)
            _draw_old_path_grey(display)
            _paint(display, ph_replan, _V_REPLAN_EX)
            _paint(display, replanned_path, _V_PATH)

        axes.clear()
        axes.imshow(_disc(display),
                    extent=[planner.x_range[0], planner.x_range[-1],
                            planner.y_range[0], planner.y_range[-1]],
                    origin='lower', cmap=cmap, vmin=0, vmax=6, alpha=0.85)
        axes.plot(start[0], start[1], 'go', markersize=8, label="Start")
        axes.plot(goal[0], goal[1], 'ro', markersize=8, label="Goal")
        axes.set_title(titles[phase](local), fontsize=14)
        axes.legend(loc='upper left')

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111)
    ax.set_aspect("equal")
    ax.set_xlabel("X [m]", fontsize=15)
    ax.set_ylabel("Y [m]", fontsize=15)

    if show_plot:
        print(f"D* search animation: {total_frames} frames")
        anime = anm.FuncAnimation(fig, update_frame, fargs=(ax,),
                                  frames=total_frames, interval=30, repeat=False)
        try:
            anime.save(gif_path, writer="pillow", fps=20)
            print(f"Search GIF saved to {gif_path}")
        except Exception as e:
            print(f"Error saving search GIF: {e}")
    else:
        test_frames = set()
        for s in offsets[:-1]:
            test_frames.update([int(s), int(s) + 1])
        for f in sorted(test_frames):
            if f < total_frames:
                update_frame(f, ax)
    plt.clf()
    plt.close()


# ---- main ------------------------------------------------------------------

def main():
    """Main process function"""

    x_lim, y_lim = MinMax(-5, 55), MinMax(-20, 25)
    sim_dir = (abs_dir_path + relative_simulations
               + "path_planning/dstar_path_planning/")
    map_path = sim_dir + "map.json"
    path_filename = sim_dir + "path.json"
    search_gif_path = sim_dir + "dstar_search.gif"
    navigate_gif_path = sim_dir + "dstar_navigate.gif"

    # ---- grid + static obstacles ----
    occ_grid = BinaryOccupancyGrid(x_lim, y_lim, resolution=0.5,
                                   clearance=1.5, map_path=map_path)

    obst_list = ObstacleList()
    obst_list.add_obstacle(Obstacle(State(x_m=10.0, y_m=15.0),
                                    length_m=10, width_m=8))
    obst_list.add_obstacle(Obstacle(State(x_m=40.0, y_m=0.0),
                                    length_m=2, width_m=10))
    occ_grid.add_object(obst_list)
    occ_grid.save_map()

    # ---- D* initial plan ----
    start, goal = (0, 0), (50, -10)
    planner = DStarPathPlanner(start, goal, map_path,
                               x_lim=x_lim, y_lim=y_lim,
                               path_filename=path_filename,
                               gif_name=None)

    initial_path = list(planner.path)
    if not initial_path:
        print("D*: no initial path found – aborting.")
        return

    grid_before_obstacle = planner.grid.copy()

    # ---- dynamic obstacle on the path ----
    block_idx = len(initial_path) * 4 // 10
    block_gx, block_gy = initial_path[block_idx]
    block_wx = planner.x_range[0] + block_gx * planner.resolution
    block_wy = planner.y_range[0] + block_gy * planner.resolution

    dyn_half_w, dyn_half_h = 1.5, 5.0
    dyn_cells = _build_obstacle_cells(planner, block_wx, block_wy,
                                      dyn_half_w, dyn_half_h)

    robot_step_at_detection = max(1, block_idx // 2)

    # ---- inject & replan ----
    planner.update_obstacles(dyn_cells)
    grid_after_obstacle = planner.grid.copy()

    current_grid_idx = (initial_path[robot_step_at_detection][1],
                        initial_path[robot_step_at_detection][0])
    replanned_tail = planner.replan(current_pos_idx=current_grid_idx)
    replan_explored = list(planner.replan_explored_nodes)
    initial_explored = list(
        planner.explored_nodes[:len(planner.explored_nodes) - len(replan_explored)]
    )

    # Build full replanned path: initial prefix (start → detection) + replanned tail
    initial_prefix = initial_path[:robot_step_at_detection]
    if replanned_tail:
        # Deduplicate the junction point if present in both segments
        if initial_prefix and replanned_tail[0] == initial_prefix[-1]:
            replanned_path = initial_prefix + replanned_tail[1:]
        else:
            replanned_path = initial_prefix + replanned_tail
    else:
        replanned_path = replanned_tail

    # ---- 1) search GIF ----
    _build_search_gif(planner, initial_path, initial_explored,
                      grid_before_obstacle, grid_after_obstacle,
                      replan_explored, replanned_path,
                      start, goal, search_gif_path)

    # ---- 2) navigation GIF (car following) ----
    # Both courses are red (standard look).  The _ReplanPurePursuit
    # controller manages which course is drawn and in what style:
    #   before replan → initial course as red dots
    #   after  replan → initial course as dotted grey, replanned as red dots
    initial_course = _path_to_course(planner, initial_path, color='r')
    replanned_course = _path_to_course(planner, replanned_path, color='r') \
        if replanned_path else initial_course

    # Dynamic obstacle as a visual Obstacle for the car scene
    dyn_obst_list = ObstacleList()
    dyn_obst_list.add_obstacle(
        Obstacle(State(x_m=block_wx, y_m=block_wy),
                 length_m=dyn_half_w, width_m=dyn_half_h)
    )

    # The switch point in world coords (where the robot "detects" the wall)
    det_gx, det_gy = initial_path[robot_step_at_detection]
    switch_wx = planner.x_range[0] + det_gx * planner.resolution
    switch_wy = planner.y_range[0] + det_gy * planner.resolution

    vis = GlobalXYVisualizer(x_lim, y_lim, TimeParameters(span_sec=20),
                             show_zoom=False, gif_name=navigate_gif_path)

    # Static obstacles (visual)
    vis.add_object(obst_list)

    # NOTE: courses are NOT added to vis directly — the controller's draw()
    # handles all course rendering so it can swap styles at replan time.

    # Vehicle with the replan-aware controller
    spec = VehicleSpecification()
    replan_ctrl = _ReplanPurePursuit(
        spec, initial_course, replanned_course,
        switch_wx, switch_wy, switch_radius=3.0,
        dynamic_obst_list=dyn_obst_list,
    )
    vehicle = FourWheelsVehicle(State(color=spec.color), spec,
                                controller=replan_ctrl, show_zoom=False)
    vis.add_object(vehicle)

    if not show_plot:
        vis.not_show_plot()
    vis.draw()


if __name__ == "__main__":
    main()

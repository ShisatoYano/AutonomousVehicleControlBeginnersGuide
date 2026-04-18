"""
mpc_path_tracking.py

Path tracking simulation using the do-mpc–based MPC controller.
Mirrors the interface and structure of mppi_path_tracking.py exactly —
only the controller class and its parameters differ.

Visualises:
  - The cubic-spline reference course
  - The four-wheels vehicle animated along the track
  - The MPC predicted (optimal) trajectory at each step  (dashed red line)

Usage
-----
    python mpc_path_tracking.py          # show animated window + save GIF
    python mpc_path_tracking.py --no-gif # window only, skip GIF encoding
"""

import sys
from pathlib import Path

# ── Path setup (identical pattern to mppi_path_tracking.py) ───────────────
abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"
sys.path.append(abs_dir_path + relative_path + "visualization")
sys.path.append(abs_dir_path + relative_path + "state")
sys.path.append(abs_dir_path + relative_path + "vehicle")
sys.path.append(abs_dir_path + relative_path + "course/cubic_spline_course")
sys.path.append(abs_dir_path + relative_path + "control/mpc") 

from global_xy_visualizer import GlobalXYVisualizer
from min_max import MinMax
from time_parameters import TimeParameters
from vehicle_specification import VehicleSpecification
from state import State
from four_wheels_vehicle import FourWheelsVehicle
from cubic_spline_course import CubicSplineCourse
from mpc_controller import MpcController              # ← do-mpc controller

show_plot = True


def main():
    # ── Visualiser window  ────────────────────────────────────────────────
    # Same course extents and time span as the MPPI script so results are
    # directly comparable side-by-side.
    x_lim, y_lim = MinMax(-5, 55), MinMax(-20, 25)
    gif_path = str(Path(__file__).absolute().parent / "mpc_path_tracking.gif")

    vis = GlobalXYVisualizer(
        x_lim, y_lim, TimeParameters(span_sec=25), gif_name=gif_path
    )

    # ── Reference course  ─────────────────────────────────────────────────
    # Identical waypoints to mppi_path_tracking.py so the comparison is fair.
    course = CubicSplineCourse(
        [0.0, 10.0, 25, 40, 50],
        [0.0,  4.0, -12, 20, -13],
        20,
    )
    vis.add_object(course)

    # ── Vehicle spec & initial state  ─────────────────────────────────────
    spec  = VehicleSpecification(area_size=20.0)
    state = State(color=spec.color)

    # ── MPC controller  ───────────────────────────────────────────────────
    # Parameters are chosen to be comparable to the MPPI configuration:
    #   delta_t and horizon give a similar prediction window (≈ 2 s).
    #   Stage / terminal cost weights mirror the MPPI weights exactly so
    #   both controllers optimise the same objective shape.
    #
    # MPC-specific knobs not present in MPPI:
    #   control_cost_weight    – penalises large raw inputs (δ², a²)
    #   smoothness_cost_weight – penalises Δu changes (do-mpc rterm)
    #   v_min / v_max          – hard speed bounds enforced by IPOPT
    #   ipopt_max_iter         – solver iteration budget
    mpc = MpcController(
        spec,
        course,
        color                  = "g",      # green dashed predicted trajectory
        delta_t                = 0.1,      # same as MPPI
        horizon_step_T         = 20,       # ≈ 2 s look-ahead  (MPPI uses 22)
        stage_cost_weight      = [50.0, 50.0, 1.0, 20.0],   # [x, y, yaw, v]
        terminal_cost_weight   = [50.0, 50.0, 1.0, 20.0],   # same as MPPI
        control_cost_weight    = [1.0, 0.5],                 # [steer, accel]
        smoothness_cost_weight = [5.0, 2.0],                 # [Δsteer, Δaccel]
        max_steer_abs          = 0.523,    # rad  (~30°) – same as MPPI
        max_accel_abs          = 2.0,      # m/s²        – same as MPPI
        v_min                  = 0.0,     # m/s  (hard bound, MPC only)
        v_max                  = 15.0,     # m/s  (hard bound, MPC only)
        ipopt_max_iter         = 100,      # IPOPT iteration limit
        ipopt_print_level      = 0,        # silent solver output
        visualize_optimal_traj = True,     # draw the predicted trajectory
    )

    # ── Vehicle  ──────────────────────────────────────────────────────────
    vehicle = FourWheelsVehicle(state, spec, controller=mpc)
    vis.add_object(vehicle)

    # ── Run  ──────────────────────────────────────────────────────────────
    if not show_plot:
        vis.not_show_plot()

    vis.draw()


if __name__ == "__main__":
    main()
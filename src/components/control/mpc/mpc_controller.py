"""
mpc_controller.py 

Model Predictive Control (MPC) controller for path tracking.
Follows the standard MPC formulation: receding-horizon optimization over a
finite horizon, bicycle kinematic model (CasADi/do-mpc backend), time-varying
reference trajectory injection via TVP, and IPOPT-based NLP solving with
automatic warm-starting. Uses (steer, accel) as control input; converts to
(accel, yaw_rate) for the existing vehicle interface. Visualizes the optimal
predicted trajectory.

────────────────────
Built on the do-mpc framework (https://www.do-mpc.com), which provides:
  - do_mpc.model.Model    : symbolic bicycle kinematic model (CasADi backend)
  - do_mpc.controller.MPC : horizon formulation, NLP assembly, IPOPT solver
  - Time-varying parameters (TVP) : reference trajectory injected each step
  - set_rterm             : quadratic smoothness / rate-of-change penalty on Delta-u
 

Reference trajectory
────────────────────
Arc-length-scaled waypoints ensure the T+1 reference points are spaced
exactly v*dt apart in distance, matching the bicycle model rollout
cadence. Velocity reference uses a quadratic taper over the final
_DECEL_DIST metres so the solver sees a smooth approach to zero speed.
All horizon steps are pinned to the last waypoint once it is reached,
preventing the tiled course array from pulling the reference off-path.
 
Stopping behaviour
──────────────────
A one-way latch (_stopped flag) activates inside _STOP_DIST of the goal.
Once latched, the controller commands a proportional brake
(accel proportional to -speed) until the vehicle is stationary, then
holds zero.  The latch never resets, preventing re-entry into the solver
after arrival.
"""

import math
import time
import warnings
import numpy as np
import do_mpc
import casadi as ca


# ── tuning knobs ────────────────────────────────────────────────────────────
_STOP_DIST  = 2.0    # latch outputs to zero inside this radius (never exits)
_BRAKE_DIST = 5.0    # quadratic brake ramp starts here
_DECEL_DIST = 15.0   # reference velocity taper starts here


class _StateView:
    """Minimal duck-type of the real State class for course queries."""

    def __init__(self, x_m, y_m, yaw_rad=0.0, speed_mps=0.0):
        self.x_m = x_m; self.y_m = y_m
        self.yaw_rad = yaw_rad; self.speed_mps = speed_mps

    def get_x_m(self):       return self.x_m
    def get_y_m(self):       return self.y_m
    def get_yaw_rad(self):   return self.yaw_rad
    def get_speed_mps(self): return self.speed_mps


class MPCController:
    """
    do-mpc based MPC path-tracking controller.
    Public interface is identical to MppiController.
    """

    def __init__(
        self,
        spec,
        course=None,
        color="r",
        delta_t: float = 0.05,
        horizon_step_T: int = 15,
        stage_cost_weight=None,
        terminal_cost_weight=None,
        control_cost_weight=None,
        smoothness_cost_weight=None,
        max_steer_abs: float = 0.523,
        max_accel_abs: float = 2.0,
        v_min: float = 0.0,          #  no reverse
        v_max: float = 20.0,
        ipopt_max_iter: int = 80,
        ipopt_print_level: int = 0,
        visualize_optimal_traj: bool = True,
    ):
        self.WHEEL_BASE_M = spec.wheel_base_m
        self.course       = course
        self._ext_x       = None
        self._ext_y       = None
        self._ext_yaw     = None
        self._ext_spd     = None
        self._ext_arc     = None
        self._n_course    = 0
        self.DRAW_COLOR   = color
        self.delta_t      = delta_t
        self.T            = horizon_step_T

        self._sw  = np.asarray(stage_cost_weight    or [50., 50., 1., 20.])
        tw_default        = self._sw * np.array([2., 2., 2., 4.]) # speed weight on terminal cost
        self._tw  = np.asarray(terminal_cost_weight or tw_default)
        self._cw  = np.asarray(control_cost_weight  or [1.0, 0.5])
        self._smw = np.asarray(smoothness_cost_weight or [5.0, 2.0])

        self.max_steer = max_steer_abs
        self.max_accel = max_accel_abs
        self.v_min     = v_min
        self.v_max     = v_max

        self._ipopt_opts = {
            "ipopt.max_iter":    ipopt_max_iter,
            "ipopt.print_level": ipopt_print_level,
            "ipopt.sb":          "yes",
            "print_time":        0,
        }

        self.target_accel_mps2   = 0.0
        self.target_steer_rad    = 0.0
        self.target_yaw_rate_rps = 0.0
        self.target_speed_mps    = 0.0
        self.optimal_trajectory  = None
        self.solve_time_ms       = 0.0
        self.solver_success      = False
        self.visualize_optimal_traj = visualize_optimal_traj

        self._current_ref       = np.zeros((4, self.T + 1))
        self._prev_waypoint_idx = 0
        self._stopped           = False   # latches True at goal; never resets

        self._build_arclength_table()
        self._model       = self._build_model()
        self._mpc         = self._build_mpc(self._model)
        self._initialized = False

    # ------------------------------------------------------------------ #
    #  do-mpc model                                                       #
    # ------------------------------------------------------------------ #
    def _build_model(self):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            model = do_mpc.model.Model("discrete")

        model.set_variable("_x", "px")
        model.set_variable("_x", "py")
        model.set_variable("_x", "yaw")
        model.set_variable("_x", "vel")
        model.set_variable("_u", "steer")
        model.set_variable("_u", "accel")
        model.set_variable("_tvp", "ref_x")
        model.set_variable("_tvp", "ref_y")
        model.set_variable("_tvp", "ref_yaw")
        model.set_variable("_tvp", "ref_vel")

        yaw_err = ca.atan2(
            ca.sin(model.x["yaw"] - model.tvp["ref_yaw"]),
            ca.cos(model.x["yaw"] - model.tvp["ref_yaw"]),
        )
        model.set_expression("yaw_err", yaw_err)

        L  = self.WHEEL_BASE_M
        dt = self.delta_t
        model.set_rhs("px",  model.x["px"]  + model.x["vel"] * ca.cos(model.x["yaw"]) * dt)
        model.set_rhs("py",  model.x["py"]  + model.x["vel"] * ca.sin(model.x["yaw"]) * dt)
        model.set_rhs("yaw", model.x["yaw"] + model.x["vel"] / L * ca.tan(model.u["steer"]) * dt)
        model.set_rhs("vel", model.x["vel"] + model.u["accel"] * dt)
        model.setup()
        return model

    # ------------------------------------------------------------------ #
    #  do-mpc controller                                                  #
    # ------------------------------------------------------------------ #
    def _build_mpc(self, model):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            mpc = do_mpc.controller.MPC(model)

        mpc.set_param(
            n_horizon           = self.T,
            t_step              = self.delta_t,
            n_robust            = 0,
            store_full_solution = True,
            nlpsol_opts         = self._ipopt_opts,
        )

        tvp_template = mpc.get_tvp_template()

        def tvp_fun(t_now):
            for k in range(self.T + 1):
                tvp_template["_tvp", k, "ref_x"]   = float(self._current_ref[0, k])
                tvp_template["_tvp", k, "ref_y"]   = float(self._current_ref[1, k])
                tvp_template["_tvp", k, "ref_yaw"] = float(self._current_ref[2, k])
                tvp_template["_tvp", k, "ref_vel"] = float(self._current_ref[3, k])
            return tvp_template

        mpc.set_tvp_fun(tvp_fun)

        sw  = self._sw
        tw  = self._tw
        x   = model.x
        tvp = model.tvp

        lterm = (
            sw[0] * (x["px"]  - tvp["ref_x"])  ** 2
          + sw[1] * (x["py"]  - tvp["ref_y"])  ** 2
          + sw[2] * model.aux["yaw_err"]        ** 2
          + sw[3] * (x["vel"] - tvp["ref_vel"]) ** 2
        )
        mterm = (
            tw[0] * (x["px"]  - tvp["ref_x"])  ** 2
          + tw[1] * (x["py"]  - tvp["ref_y"])  ** 2
          + tw[2] * model.aux["yaw_err"]        ** 2
          + tw[3] * (x["vel"] - tvp["ref_vel"]) ** 2
        )

        mpc.set_objective(lterm=lterm, mterm=mterm)
        mpc.set_rterm(
            steer = float(self._cw[0] + self._smw[0]),
            accel = float(self._cw[1] + self._smw[1]),
        )

        mpc.bounds["lower", "_u", "steer"] = -self.max_steer
        mpc.bounds["upper", "_u", "steer"] =  self.max_steer
        mpc.bounds["lower", "_u", "accel"] = -self.max_accel
        mpc.bounds["upper", "_u", "accel"] =  self.max_accel
        mpc.bounds["lower", "_x", "vel"]   =  self.v_min
        mpc.bounds["upper", "_x", "vel"]   =  self.v_max

        mpc.setup()
        return mpc

    # ------------------------------------------------------------------ #
    #  Arc-length table                                                   #
    # ------------------------------------------------------------------ #
    def _build_arclength_table(self) -> None:
        if not self.course:
            return
        n = 0
        xs, ys, yaws, speeds = [], [], [], []
        while True:
            try:
                xs.append(self.course.point_x_m(n))
                ys.append(self.course.point_y_m(n))
                yaws.append(self.course.point_yaw_rad(n))
                speeds.append(self.course.point_speed_mps(n))
                n += 1
            except Exception:
                break
        if n == 0:
            return

        xs     = np.array(xs);     ys    = np.array(ys)
        yaws   = np.array(yaws);   speeds = np.array(speeds)
        seg    = np.hypot(np.diff(xs), np.diff(ys))
        arc    = np.concatenate([[0.0], np.cumsum(seg)])
        total  = arc[-1]

        self._n_course = n
        self._ext_x    = np.tile(xs,     3)
        self._ext_y    = np.tile(ys,     3)
        self._ext_yaw  = np.tile(yaws,   3)
        self._ext_spd  = np.tile(speeds, 3)
        self._ext_arc  = np.concatenate([arc, arc + total, arc + 2 * total])

    # ------------------------------------------------------------------ #
    #  Reference trajectory                                              #
    # ------------------------------------------------------------------ #
    def _get_reference_trajectory(self, x0: float, y0: float,
                                   current_speed: float) -> np.ndarray:
        """
        Build T+1 reference waypoints.

        once best_idx reaches the last course point, every
        remaining horizon step is pinned to that last waypoint
        with ref_vel = 0.  This prevents the tiled region from
        pulling the reference (and therefore the robot) off the
        actual path near the goal.

        quadratic velocity taper over _DECEL_DIST = 15 m.
        """
        if not self.course or self._ext_arc is None:
            return np.zeros((4, self.T + 1))

        n        = self._n_course
        dt       = self.delta_t
        v_ref    = max(abs(current_speed), 1.0)
        step_len = v_ref * dt

        goal_x = self._ext_x[n - 1]
        goal_y = self._ext_y[n - 1]

        # ── nearest forward index (clamped to first tile) ─────────────────
        WINDOW   = 60
        best_idx = self._prev_waypoint_idx
        best_d   = float("inf")
        for off in range(WINDOW):
            ci = self._prev_waypoint_idx + off
            if ci >= n:
                break
            d  = (self._ext_x[ci] - x0) ** 2 + (self._ext_y[ci] - y0) ** 2
            if d < best_d:
                best_d   = d
                best_idx = ci

        best_idx = min(best_idx, n - 1)   # hard clamp
        self._prev_waypoint_idx = best_idx

        s0  = self._ext_arc[best_idx]
        ref = np.zeros((4, self.T + 1))

        for k in range(self.T + 1):
            # already at last waypoint: pin all future steps
            if best_idx >= n - 1:
                ref[0, k] = self._ext_x[n - 1]
                ref[1, k] = self._ext_y[n - 1]
                ref[2, k] = self._ext_yaw[n - 1]
                ref[3, k] = 0.0
                continue

            target_s     = s0 + k * step_len
            search_start = best_idx
            search_end   = n                      # stay within first tile
            diffs        = np.abs(self._ext_arc[search_start:search_end] - target_s)
            ki           = search_start + int(np.argmin(diffs))
            ki           = min(ki, n - 1)

            ref[0, k] = self._ext_x[ki]
            ref[1, k] = self._ext_y[ki]
            ref[2, k] = self._ext_yaw[ki]

            # quadratic taper
            dist_to_goal = np.hypot(self._ext_x[ki] - goal_x,
                                    self._ext_y[ki] - goal_y)
            if dist_to_goal < _DECEL_DIST:
                t         = dist_to_goal / _DECEL_DIST  
                ref[3, k] = self._ext_spd[ki] * (t * t)
            else:
                ref[3, k] = self._ext_spd[ki]

        return ref

    # ------------------------------------------------------------------ #
    #  Public update                                                      #
    # ------------------------------------------------------------------ #
    def update(self, state, time_s: float) -> None:
        if not self.course:
            self.target_accel_mps2   = 0.0
            self.target_yaw_rate_rps = 0.0
            self.target_steer_rad    = 0.0
            self.target_speed_mps    = state.get_speed_mps()
            self.optimal_trajectory  = None
            return

        px  = float(state.get_x_m())
        py  = float(state.get_y_m())
        yaw = float(state.get_yaw_rad())
        vel = float(state.get_speed_mps())
        x0  = np.array([[px], [py], [yaw], [vel]])

        self._current_ref = self._get_reference_trajectory(px, py, vel)

        dist_to_goal = float("inf")
        if self._n_course > 0:
            gx = self._ext_x[self._n_course - 1]
            gy = self._ext_y[self._n_course - 1]
            dist_to_goal = np.hypot(px - gx, py - gy)

            # Latch it, once inside _STOP_DIST
            if dist_to_goal < _STOP_DIST:
                self._stopped = True

            if self._stopped:
                # Actively brake until truly stopped, then hold zero.
                # Setting accel=0 leaves momentum; we must command negative
                # accel proportional to current speed until vel ~ 0.
                if abs(vel) > 0.05:
                    self.target_accel_mps2 = max(-self.max_accel,
                                                  -self.max_accel * min(abs(vel), 1.0))
                else:
                    self.target_accel_mps2 = 0.0
                self.target_steer_rad    = 0.0
                self.target_yaw_rate_rps = 0.0
                self.target_speed_mps    = 0.0
                self.optimal_trajectory  = None
                return

        if not self._initialized:
            self._mpc.x0 = x0
            self._mpc.set_initial_guess()
            self._initialized = True

        t_start = time.perf_counter()
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            try:
                u0 = self._mpc.make_step(x0)
                self.solver_success = True
            except Exception:
                u0 = np.zeros((2, 1))
                self.solver_success = False
        self.solve_time_ms = (time.perf_counter() - t_start) * 1e3

        steer0 = float(np.clip(np.asarray(u0).flat[0], -self.max_steer, self.max_steer))
        accel0 = float(np.clip(np.asarray(u0).flat[1], -self.max_accel, self.max_accel))

        # Quadratic brake ramp override — force deceleration near goal
        if dist_to_goal < _BRAKE_DIST:
            brake_scale = (dist_to_goal / _BRAKE_DIST) ** 2   
            max_brake   = -self.max_accel * (1.0 - brake_scale)
            accel0      = min(accel0, max_brake)
            # Once essentially stopped, hold at exactly zero
            if vel < 0.1:
                accel0 = 0.0

        # when nearly stopped, never command negative accel (no reverse)
        # This is unconditional: v_min=0 prevents the state going negative but
        # one solver step of negative accel still causes a backward lurch.
        if abs(vel) < 0.3:
            accel0 = max(accel0, 0.0) if dist_to_goal > _BRAKE_DIST else 0.0

        self.target_steer_rad  = steer0
        self.target_accel_mps2 = accel0

        v0 = state.get_speed_mps()
        self.target_yaw_rate_rps = (
            0.0 if abs(v0) < 1e-9
            else v0 / self.WHEEL_BASE_M * math.tan(steer0)
        )
        self.target_speed_mps = v0

        if self.visualize_optimal_traj:
            try:
                px_pred = [float(np.asarray(self._mpc.opt_x_num["_x", k, 0, "px"]).flat[0])
                           for k in range(self.T + 1)]
                py_pred = [float(np.asarray(self._mpc.opt_x_num["_x", k, 0, "py"]).flat[0])
                           for k in range(self.T + 1)]
                self.optimal_trajectory = (px_pred, py_pred)
            except Exception:
                try:
                    x_pred = np.array(self._mpc.opt_x_num["_x"]).reshape(self.T + 1, -1)
                    self.optimal_trajectory = (x_pred[:, 0].tolist(), x_pred[:, 1].tolist())
                except Exception:
                    self.optimal_trajectory = None
        else:
            self.optimal_trajectory = None

    # ------------------------------------------------------------------ #
    #  Getters                                                            #
    # ------------------------------------------------------------------ #
    def get_target_accel_mps2(self)   -> float: return self.target_accel_mps2
    def get_target_steer_rad(self)    -> float: return self.target_steer_rad
    def get_target_yaw_rate_rps(self) -> float: return self.target_yaw_rate_rps

    # ------------------------------------------------------------------ #
    #  Visualisation                                                      #
    # ------------------------------------------------------------------ #
    def draw(self, axes, elems) -> None:
        if self.visualize_optimal_traj and self.optimal_trajectory:
            x_list, y_list = self.optimal_trajectory
            (line,) = axes.plot(
                x_list, y_list,
                color=self.DRAW_COLOR, linewidth=2.0,
                linestyle="-", alpha=0.9, label="MPC trajectory",
            )
            elems.append(line)
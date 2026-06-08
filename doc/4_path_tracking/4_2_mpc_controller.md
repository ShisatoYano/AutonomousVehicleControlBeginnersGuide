# 5. MPC Controller

In this chapter, the MPC (Model Predictive Control) path tracking controller class is implemented. This class implements the MPC path tracking algorithm, which computes a steering angle and acceleration command by solving an **optimization problem** over a finite prediction horizon at every time step.

Before getting into the code, let's get some basic understanding behind the algorithm:

### **Model Predictive Control (MPC)**

- Also known as **Receding Horizon Control**.
- Widely used in autonomous vehicles, robotics, and industrial process control.
- At every time step, MPC solves an Optimal Control Problem (OCP) over a future horizon of **N steps**, then applies only the **first control action** and repeats the process at the next step.

**The key idea is**: instead of reacting to the current error (like Stanley or PID), MPC **predicts** where the vehicle will be over the next N steps and finds the control sequence that minimises a cost function while respecting the hard constraints.

---

**Advantages over Stanley / Pure Pursuit / PID :**

- Handles **hard constraints** natively - steering limits, acceleration limits, and speed bounds are enforced by the solver, not violated and then clipped(like in geometric controllers).
- Plans **N steps ahead** - can anticipate future curves before the vehicle reaches them and adjust the control input according to that.
- Combines tracking error, control effort, and smoothness of control inputs in one unified cost function.
- Natively handles variable speed profiles along the course.

**Disadvantages:**

- Computationally heavier : requires solving an NLP (Non-Linear Program) at every step via IPOPT.
- Requires tuning of multiple cost weights.
- Sensitive to model mismatch - if the internal vehicle model differs from the real vehicle, tracking performance degrades.

---

#### Libraries used: 

1. **CasADi** is a symbolic math library. When you write $p_x + v * cos(\theta) * dt$ in CasADi, it does not compute a number, it stores the formula as an expression tree, like algebra. The reason this matters is that IPOPT needs the derivative of every equation at every iteration. CasADi computes those derivatives analytically and exactly (automatic differentiation), the same way we'd differentiate by hand. Without this, we'd have to approximate derivatives numerically, which is slow and inaccurate. In the MPC code, every `model.set_rhs()` call is for handing CasADi a symbolic expression to work with.

2. **do-mpc** is the MPC framework that sits on top of CasADi. It handles all the boilerplate - assembling the horizon loop, managing the decision variable layout, calling the `tvp_fun()` before each solve to inject the latest reference trajectory, passing everything to *IPOPT*, reading the solution, and shifting the warm start for next time. Without do-mpc we'd write all of that by hand. With do-mpc, `make_step(x0)` does all of it in one call.

---

#### Important terms: 

1. **IPOPT** (Interior Point OPTimizer) is the actual numerical solver - the thing that finds the answer. It receives a cost value and its gradient, constraint violations and their Jacobians, from CasADi at each iteration. It then takes a step using the interior point method (imagine a ball rolling down a valley while staying inside a fence), and repeats until the gradient is near zero ($\epsilon$) and all constraints are satisfied. It returns the optimal [steer, accel] sequence.

2. A Nonlinear Program (NLP) is the class of math problem MPC belongs to. It is *"nonlinear"* because the bicycle model contains tan(steer) and cos(yaw) curved functions. That makes it harder than a linear problem (which has exactly one minimum you can jump to directly). NLPs can have multiple local minima, which is why warm-starting from the previous solution matters so much, and why **MPPI** (which samples globally) can sometimes find better solutions than MPC on tricky(back to back tight curves) courses.

---

## 5.1 MpcController Class

The controller class is located at:
[mpc_controller.py](/src/components/control/mpc/mpc_controller.py)

```python
"""
mpc_controller.py
"""

import math
import time
import warnings
import numpy as np
import do_mpc
import casadi as ca


class MpcController:
    """
    do-mpc-based MPC path-tracking controller.
    Public interface identical to MppiController.
    """
```

This class uses **do-mpc** as the MPC framework and **CasADi** as the symbolic algebra backend. CasADi automatically computes the exact gradients and Hessians that the IPOPT solver needs and so, no finite-difference approximations are required.

---

### 5.1.1 Constructor

```python
def __init__(self, spec, course=None, color="r",
             delta_t=0.05, horizon_step_T=15,
             stage_cost_weight=None,
             terminal_cost_weight=None,
             control_cost_weight=None,
             smoothness_cost_weight=None,
             max_steer_abs=0.523,
             max_accel_abs=2.0,
             v_min=-1.0, v_max=20.0,
             ipopt_max_iter=80,
             ipopt_print_level=0,
             visualize_optimal_traj=True):

    self.WHEEL_BASE_M = spec.wheel_base_m
    self.course       = course
    self.delta_t      = delta_t
    self.T            = horizon_step_T

    self._sw  = np.asarray(stage_cost_weight      or [50., 50., 1., 20.])
    self._tw  = np.asarray(terminal_cost_weight   or self._sw * 2.)
    self._cw  = np.asarray(control_cost_weight    or [1.0, 0.5])
    self._smw = np.asarray(smoothness_cost_weight or [5.0, 2.0])

    self.max_steer = max_steer_abs
    self.max_accel = max_accel_abs
    self.v_min     = v_min
    self.v_max     = v_max

    self.target_accel_mps2   = 0.0
    self.target_steer_rad    = 0.0
    self.target_yaw_rate_rps = 0.0
    self._prev_waypoint_idx  = 0

    self._build_arclength_table()
    self._model = self._build_model()
    self._mpc   = self._build_mpc(self._model)
```

The constructor takes a `VehicleSpecification` object and an optional `CubicSplineCourse` object. The key member variables are:

| Variable | Default | Description |
|---|---|---|
| `WHEEL_BASE_M` | from spec | Distance between front and rear axles [m] |
| `T` | 15 | Prediction horizon - number of steps looked ahead |
| `delta_t` | 0.05 | Time step for each horizon step [s] |
| `_sw` | [50, 50, 1, 20] | Stage cost weights `[w_x, w_y, w_ψ, w_v]` |
| `_tw` | 2 × `_sw` | Terminal cost weights — heavier than stage |
| `_cw` | [1.0, 0.5] | Control effort weights `[w_δ, w_a]` |
| `_smw` | [5.0, 2.0] | Smoothness weights `[w_Δδ, w_Δa]` |
| `max_steer` | 0.523 rad | Hard steering bound (~30°) |
| `max_accel` | 2.0 m/s² | Hard acceleration bound |
| `v_min / v_max` | -1 / 20 m/s | Hard speed bounds |
| `target_accel_mps2` | 0.0 | Computed acceleration command [m/s²] |
| `target_steer_rad` | 0.0 | Computed steering angle command [rad] |
| `target_yaw_rate_rps` | 0.0 | Computed yaw rate command [rad/s] |

---

## 5.2 Algorithm Background

### 5.2.1 State and Control Vectors

MPC operates on a **state vector** $x$ and a **control vector** $u$:

$$
x_t =
\begin{bmatrix}
p_x & p_y & \psi & v
\end{bmatrix}^{T}
$$

$$
u_t =
\begin{bmatrix}
\delta & a
\end{bmatrix}^{T}
$$



At each time step, MPC finds the sequence $U = {u₀, u₁, …, u_{N-1}}$ that minimises the total cost $J$ over the horizon.

---

### 5.2.2 Vehicle Model - Discrete Bicycle Kinematics

MPC uses an internal motion model to predict how the vehicle will move in response to each control. The **kinematic bicycle model** is used:

$$
\begin{aligned}
p_{x,t+1} &= p_{x,t} + v_t \cos(\psi_t)\,\Delta t, \\
p_{y,t+1} &= p_{y,t} + v_t \sin(\psi_t)\,\Delta t, \\
\psi_{t+1} &= \psi_t + \frac{v_t}{L}\tan(\delta_t)\,\Delta t, \\
v_{t+1} &= v_t + a_t\,\Delta t.
\end{aligned}
$$

where $L$ is the wheelbase and $Δt$ is the time step. This is the same kinematic relationship used in the other controller also, only the key difference is that MPC applies it **N times symbolically** to predict the full future trajectory.

CasADi writes these equations as symbolic expressions, meaning it can automatically compute exact derivatives, the same way you would differentiate by hand, but done by the computer.

---

### 5.2.3 The Optimization Problem

At every time step, MPC solves the optimization problem:

$$
\min_{U}\quad J =
\sum_{t=0}^{N-1}
\ell(x_t,u_t,u_{t-1})
+
\phi(x_N)
$$

subject to

$$
x_{t+1}=f(x_t,u_t)
$$

$$
|\delta_t| \le \delta_{\max}
$$

$$
|a_t| \le a_{\max}
$$

$$
v_{\min} \le v_t \le v_{\max}
$$

where:

- $x_{t+1}=f(x_t,u_t)$ enforces the bicycle model dynamics at every prediction step.
- $|\delta_t| \le \delta_{\max}$ imposes a hard steering constraint.
- $|a_t| \le a_{\max}$ imposes a hard acceleration constraint.
- $v_{\min} \le v_t \le v_{\max}$ imposes hard speed limits.

The constraints are **hard constraints**, meaning the optimizer (e.g., IPOPT) enforces them directly during optimization rather than clipping the control output afterward.

---

### 5.2.4 Stage Cost $\ell(x_t, u_t, u_{t-1})$

The stage cost is evaluated at every step $t = 0,\ldots,N-1$ of the horizon. It has three parts:

#### 1. Tracking Error

Penalises deviation from the reference trajectory:

$$
\ell_{\text{track}}
=
w_x (p_{x,t} - p_{x,r})^2
+
w_y (p_{y,t} - p_{y,r})^2
+
w_\psi
\left[
\mathrm{atan2}
\left(
\sin(\psi_t-\psi_r),
\cos(\psi_t-\psi_r)
\right)
\right]^2
+
w_v (v_t-v_r)^2
$$

The heading error uses

$$
\mathrm{atan2}
\left(
\sin(\psi_t-\psi_r),
\cos(\psi_t-\psi_r)
\right)
$$

rather than a direct subtraction. This maps the error into:

$$
(-\pi,\pi]
$$

and remains smooth when the angle wraps around $\pm\pi$.

---

#### 2. Control Effort

Penalises large control inputs:

$$
\ell_{\text{effort}}
=
w_\delta \,\delta_t^2
+
w_a\,a_t^2
$$

---

#### 3. Smoothness

Penalises rapid changes between consecutive control inputs:

$$
\ell_{\text{smooth}}
=
w_{\Delta\delta}
(\delta_t-\delta_{t-1})^2
+
w_{\Delta a}
(a_t-a_{t-1})^2
$$

This is implemented via do-mpc's `set_rterm()` which automatically adds this term at every horizon step. It prevents the controller from producing jerky steering commands.

In code, effort and smoothness are combined into a single `rterm` weight per channel:

```python
mpc.set_rterm(
    steer = control_cost_weight[0] + smoothness_cost_weight[0],
    accel = control_cost_weight[1] + smoothness_cost_weight[1],
)
```

---

### 5.2.5 Terminal Cost $\phi(x_N)$

The terminal cost is evaluated only at the **last step** $t = N$ of the horizon:

$$
\phi(x_N)
=
W_x (p_{x,N} - p_{x,rN})^2
+
W_y (p_{y,N} - p_{y,rN})^2
+
W_\psi
\left[
\mathrm{atan2}
\left(
\sin(\psi_N-\psi_{rN}),
\cos(\psi_N-\psi_{rN})
\right)
\right]^2
+
W_v (v_N-v_{rN})^2
$$

The terminal weights $W$ are set heavier than the stage weights (default: $W = 2 × w$). Without a terminal cost, the optimizer has no incentive to reach a good state at the end of the horizon, it can "borrow" performance from future steps that it cannot see.

---

### 5.2.6 The Receding Horizon Principle

MPC solves for the full sequence ${u₀, u₁, …, u_{N-1}}$ but **only applies $u₀$** - the first control action. At the next time step, the horizon shifts forward by one step, the current state is re-measured, and the problem is solved again from scratch. This is why it is called a **receding horizon controller**.

At each control cycle, MPC solves an optimization problem over the prediction horizon, but only the first control action is applied:

$$
U^*
=
\left\{
u_0^*,\,u_1^*,\,\ldots,\,u_{N-1}^*
\right\}
$$

but only

$$
u(k)=u_0^*
$$

is applied to the vehicle.

At the next sampling instant, the optimization is solved again using the updated state estimate:

$$
x(k+1)
$$

resulting in a new optimal control sequence.

This strategy is known as the **receding horizon** (or **moving horizon**) principle.

**Warm starting**: the previous solution is shifted by one step and used as the initial guess for the next solve. This means *IPOPT* starts close to the true optimum and converges in far fewer iterations.

---

## 5.3 Private Methods

### 5.3.1 `_build_arclength_table()`

```python
def _build_arclength_table(self):
    xs, ys, yaws, speeds = [], [], [], []
    n = 0
    while True:
        try:
            xs.append(self.course.point_x_m(n))
            ...
            n += 1
        except Exception:
            break

    seg = np.hypot(np.diff(xs), np.diff(ys))
    arc = np.concatenate([[0.0], np.cumsum(seg)])
    total = arc[-1]

    self._ext_x   = np.tile(xs,   3)
    self._ext_arc = np.concatenate([arc, arc + total, arc + 2 * total])
```

This method is called once in `__init__()`. It samples every course point, computes the **cumulative arc-length** (total distance along the curve from the start), and tiles the arrays three times.

The tiling handles closed or looping courses: when a vehicle near the end of the course needs to look $T$ steps ahead, the indices wrap around to the beginning. Without tiling, you would need modular arithmetic at every reference lookup; with tiling, the arrays are simply longer and you index linearly.

---

### 5.3.2 `_build_model()`

```python
def _build_model(self):
    model = do_mpc.model.Model("discrete")

    model.set_variable("_x",   "px")
    model.set_variable("_x",   "py")
    model.set_variable("_x",   "yaw")
    model.set_variable("_x",   "vel")
    model.set_variable("_u",   "steer")
    model.set_variable("_u",   "accel")
    model.set_variable("_tvp", "ref_x")
    model.set_variable("_tvp", "ref_y")
    model.set_variable("_tvp", "ref_yaw")
    model.set_variable("_tvp", "ref_vel")

    yaw_err = ca.atan2(
        ca.sin(model.x["yaw"] - model.tvp["ref_yaw"]),
        ca.cos(model.x["yaw"] - model.tvp["ref_yaw"]),
    )
    model.set_expression("yaw_err", yaw_err)

    model.set_rhs("px",  model.x["px"]  + model.x["vel"] * ca.cos(model.x["yaw"]) * dt)
    model.set_rhs("py",  model.x["py"]  + model.x["vel"] * ca.sin(model.x["yaw"]) * dt)
    model.set_rhs("yaw", model.x["yaw"] + model.x["vel"] / L * ca.tan(model.u["steer"]) * dt)
    model.set_rhs("vel", model.x["vel"] + model.u["accel"] * dt)

    model.setup()
    return model
```

This method declares the **symbolic bicycle model** using do-mpc's `Model` class. Every variable registered here becomes a CasADi symbolic expression - the same algebra that a mathematician would write, but executable and automatically differentiable.

Three types of symbolic variables are registered:

| Variable type | Names | Description |
|---|---|---|
| `_x`   (state)          | `px, py, yaw, vel`            | The four vehicle states |
| `_u`   (control)        | `steer, accel`                | The two control inputs |
| `_tvp` (time-varying)   | `ref_x, ref_y, ref_yaw, ref_vel` | Reference trajectory changes each step |

The **time-varying parameter (TVP)** is the key do-mpc feature that allows the reference trajectory to be updated every step. do-mpc calls `tvp_fun()` automatically before each solve and fills in the reference values for all `N+1` horizon steps.

The `yaw_err` auxiliary expression is registered separately so it can be referenced cleanly in the cost function. It uses `atan2(sin(·), cos(·))` for the same reason as in the cost formulation above.

---

### 5.3.3 `_build_mpc(model)`

```python
def _build_mpc(self, model):
    mpc = do_mpc.controller.MPC(model)
    mpc.set_param(n_horizon=self.T, t_step=self.delta_t,
                  n_robust=0, store_full_solution=True,
                  nlpsol_opts=self._ipopt_opts)

    # TVP function — closure over self._current_ref
    tvp_template = mpc.get_tvp_template()
    def tvp_fun(t_now):
        for k in range(self.T + 1):
            tvp_template["_tvp", k, "ref_x"]   = float(self._current_ref[0, k])
            tvp_template["_tvp", k, "ref_y"]   = float(self._current_ref[1, k])
            tvp_template["_tvp", k, "ref_yaw"] = float(self._current_ref[2, k])
            tvp_template["_tvp", k, "ref_vel"] = float(self._current_ref[3, k])
        return tvp_template
    mpc.set_tvp_fun(tvp_fun)

    lterm = (sw[0]*(x["px"]-tvp["ref_x"])**2 + sw[1]*(x["py"]-tvp["ref_y"])**2
           + sw[2]*model.aux["yaw_err"]**2     + sw[3]*(x["vel"]-tvp["ref_vel"])**2)
    mterm = (tw[0]*(x["px"]-tvp["ref_x"])**2 + ...)

    mpc.set_objective(lterm=lterm, mterm=mterm)
    mpc.set_rterm(steer=cw[0]+smw[0], accel=cw[1]+smw[1])

    mpc.bounds["lower", "_u", "steer"] = -self.max_steer
    mpc.bounds["upper", "_u", "steer"] =  self.max_steer
    mpc.bounds["lower", "_u", "accel"] = -self.max_accel
    mpc.bounds["upper", "_u", "accel"] =  self.max_accel
    mpc.bounds["lower", "_x", "vel"]   =  self.v_min
    mpc.bounds["upper", "_x", "vel"]   =  self.v_max

    mpc.setup()
    return mpc
```

This method assembles the full MPC optimization problem using the do-mpc API:

- `set_param()` - sets the horizon length, time step, and IPOPT options.
- `set_tvp_fun()` - registers the TVP closure so do-mpc calls it automatically before each solve.
- `set_objective(lterm, mterm)` - symbolic stage and terminal costs. `lterm` is evaluated at every step $t = 0…N-1$; `mterm` only at step $t = N$.
- `set_rterm()` - registers the smoothness/effort Δu penalty. do-mpc adds $r_i·(u_{i,t} − u_{i,t−1})²$ at every horizon step automatically.
- `bounds[]` - hard box constraints on steering, acceleration, and speed.

---

### 5.3.4 `_get_reference_trajectory(x0, y0, current_speed)`

```python
def _get_reference_trajectory(self, x0, y0, current_speed):
    v_ref    = max(abs(current_speed), 1.0)
    step_len = v_ref * self.delta_t      # arc-length between ref points

    # Forward search: find nearest index ahead of previous position
    WINDOW = 60
    best_idx = self._prev_waypoint_idx
    for off in range(WINDOW):
        ci   = self._prev_waypoint_idx + off
        dist = (self._ext_x[ci] - x0)**2 + (self._ext_y[ci] - y0)**2
        if dist < best_dist:
            best_idx = ci
    self._prev_waypoint_idx = best_idx

    # Arc-length spaced reference: ref[k] is at distance k × v × Δt ahead
    s0 = self._ext_arc[best_idx]
    for k in range(self.T + 1):
        target_s = s0 + k * step_len
        ki = argmin |ext_arc[best_idx:] - target_s|
        ref[:, k] = ext_x[ki], ext_y[ki], ext_yaw[ki], ext_spd[ki]
    return ref
```

This method builds the $(4, T+1)$ reference array that is injected into the NLP via the TVP function. It has two important design decisions:

**Forward-only search**: the nearest index is found by searching only *ahead* of `_prev_waypoint_idx`, never behind it. This prevents jumping to the wrong arc on a closed or figure-8 course.

**Arc-length spaced reference**: reference point $k$ is placed at arc-length distance $k × v × Δt$ along the course ahead of the vehicle. This ensures the reference horizon always spans exactly the distance the vehicle will travel in $N$ steps - no more, no less. Without this, the reference would "run ahead" of the vehicle on tight curves and produce a horizon that points in the wrong direction.

The formula for each reference step is:

```
target_s  =  s₀  +  k × v × Δt
ref[k]    =  course point nearest to arc-length target_s
```

where $s₀$ is the cumulative arc-length at the vehicle's current nearest point, and $v × Δt$ is the distance travelled per prediction step.

---

## 5.4 Public Methods

### 5.4.1 `update`

```python
def update(self, state, time_s):
    if not self.course: return

    px  = float(state.get_x_m())
    py  = float(state.get_y_m())
    yaw = float(state.get_yaw_rad())
    vel = float(state.get_speed_mps())
    x0  = np.array([[px], [py], [yaw], [vel]])   # (4, 1) column vector

    self._current_ref = self._get_reference_trajectory(px, py, vel)

    if not self._initialized:
        self._mpc.x0 = x0
        self._mpc.set_initial_guess()
        self._initialized = True

    u0 = self._mpc.make_step(x0)    # returns [[steer], [accel]]

    steer0 = float(np.clip(np.asarray(u0).flat[0], -self.max_steer, self.max_steer))
    accel0 = float(np.clip(np.asarray(u0).flat[1], -self.max_accel, self.max_accel))

    self.target_steer_rad    = steer0
    self.target_accel_mps2   = accel0
    self.target_yaw_rate_rps = vel / self.WHEEL_BASE_M * tan(steer0)
```

This is the main entry point called every simulation frame by `FourWheelsVehicle`. It orchestrates all private methods in order:

```
update()
  1. Extract px, py, yaw, vel from state getters
  2. Build x0 as (4,1) column vector — do-mpc's required shape
  3. Build reference trajectory via _get_reference_trajectory()
  4. First call only: set initial guess so IPOPT starts feasibly
  5. mpc.make_step(x0) — do-mpc calls tvp_fun(), then solves the NLP
  6. Extract steer0, accel0 from solution u0
  7. Compute yaw_rate = v / L * tan(steer0)   (bicycle model formula)
  8. Extract predicted trajectory from opt_x_num for draw()
```

If no course is set, the method returns early without computing anything.

**A note on $u_0$ extraction**: do-mpc returns $u_0$ as a $(2, 1)$ column vector, but the shape can vary depending on the version. Using `.flat[0]` and `.flat[1]` (rather than `[0,0]` and `[1,0]`) is robust to both `(2,1)` and `(2,)` shapes.

The yaw rate is then derived from the steering angle using the kinematic bicycle model.

$$
\omega = v * tan(δ) / L
$$

---

### 5.4.2 Getter Methods

```python
def get_target_accel_mps2(self):
    return self.target_accel_mps2

def get_target_yaw_rate_rps(self):
    return self.target_yaw_rate_rps

def get_target_steer_rad(self):
    return self.target_steer_rad
```

These three getter methods expose the computed control outputs. They are called by `FourWheelsVehicle` after each `update()` to apply the commands to the vehicle's state. The interface is identical to `StanleyController`.

---

### 5.4.3 `draw`

```python
def draw(self, axes, elems):
    if self.visualize_optimal_traj and self.optimal_trajectory:
        x_list, y_list = self.optimal_trajectory
        (line,) = axes.plot(x_list, y_list,
                            color=self.DRAW_COLOR,
                            linewidth=2.0,
                            linestyle="--",
                            alpha=0.9,
                            label="MPC trajectory")
        elems.append(line)
```

MPC controller draws the **predicted trajectory** - the optimal state sequence ${x₀*, x₁*, …, x_N*}$ from the last NLP solution. This red dashed line shows exactly where the MPC controller plans the vehicle to go over the next $N × Δt$ seconds. It is a direct visualisation of what the optimizer computed, which makes debugging tuning much easier.

The trajectory is extracted from do-mpc's `opt_x_num` structure by name:

```python
px_pred = [opt_x_num["_x", k, 0, "px"] for k in range(T + 1)]
py_pred = [opt_x_num["_x", k, 0, "py"] for k in range(T + 1)]
```

---

**Author**: Mohit Kumar

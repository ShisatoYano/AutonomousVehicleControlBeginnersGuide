# 6. MPPI Controller

In this chapter, the MPPI (Model Predictive Path Integral) path tracking controller class is implemented. This class implements the MPPI path tracking algorithm, which computes a steering angle and acceleration command by **sampling thousands of random control sequences**, rolling each one forward through the vehicle dynamics, scoring them by cost, and computing a weighted average update.

Before getting into the code, let's get some basic understanding behind the algorithm:

### **Model Predictive Path Integral (MPPI)**

- Introduced by Williams et al. (2017) at Georgia Tech.
- Belongs to the family of **stochastic optimal control** methods.
- Instead of solving an optimization problem algebraically (like MPC), MPPI samples K random perturbations around a nominal control sequence, simulates each one, and combines them using **information-theoretic weighting**.

The key idea is: run $K$ imagined futures in parallel, score each by how well it tracks the path, then take a weighted combination - low-cost futures get high weight, high-cost futures get low weight.

1. **Sampling** - draw K random noise sequences from a Gaussian distribution
2. **Rollout** - simulate the vehicle forward T steps for each sample
3. **Weighting** - assign higher weight to samples with lower trajectory cost
4. **Update** - shift the nominal control sequence toward the best samples

<p align="center">
  <img src="./mppi.png" alt="MPPI overview" width="400">
</p>

Ref: https://dilithjay.com/blog/mppi

**Advantages over MPC:**

- No solver required - the update is a simple weighted sum.
- Naturally parallelisable over K - runs efficiently on a GPU.
- Handles **non-convex, non-smooth** cost landscapes where gradient-based solvers struggle.
- Global exploration through stochastic sampling - can escape local minima.

**Disadvantages:**

- Constraints are **soft only** - satisfied by increasing cost, never hard-blocked(like MPC where hard constraints are inforced).
- Solution quality depends on K - more samples give a better approximation but cost more compute.
- Tuning parameters($λ$, $K$, $σ$) requires experimentation with no systematic design method.
- Can be noisy at low K values, producing jittery control.

---

## 6.1 MppiController Class

The controller class is located at:
[mppi_controller.py](/src/components/control/mppi/mppi_controller.py)

```python
"""
mppi_controller.py

Author: Shisato Yano
"""

import math, sys
from pathlib import Path
from math import atan2, cos, sin, tan
import numpy as np

from state import State


class MppiController:
    """
    MPPI path-tracking controller aligned with standard formulation:
    warm start (u_prev), exploitation/exploration sampling, stage + terminal cost,
    control cost term (param_gamma * u.T @ inv(Sigma) @ v), and optional smoothing.
    """
```

This class imports trigonometric functions from Python's `math` module and `numpy` for vectorised sampling and matrix operations. It also imports the `State` class to use its `motion_model` static method for simulating the vehicle forward during each sample rollout.

---

### 6.1.1 Constructor

```python
def __init__(self, spec, course=None, color="g",
             delta_t=0.05, horizon_step_T=20,
             number_of_samples_K=256,
             param_exploration=0.0,
             param_lambda=50.0, param_alpha=1.0,
             sigma_steer=0.1, sigma_accel=0.5,
             max_steer_abs=0.523, max_accel_abs=2.0,
             stage_cost_weight=None,
             terminal_cost_weight=None,
             moving_average_window=0,
             visualize_optimal_traj=True,
             visualize_sampled_trajs=True):

    self.WHEEL_BASE_M      = spec.wheel_base_m
    self.T                 = horizon_step_T
    self.K                 = number_of_samples_K
    self.param_lambda      = max(1e-6, param_lambda)
    self.param_gamma       = param_lambda * (1.0 - param_alpha)
    self.Sigma             = np.array([[sigma_steer**2, 0.0],
                                       [0.0, sigma_accel**2]])
    self.stage_cost_weight = np.asarray(stage_cost_weight
                                        or [50.0, 50.0, 1.0, 20.0])
    self.terminal_cost_weight = np.asarray(terminal_cost_weight
                                           or self.stage_cost_weight.copy())
    self.u_prev            = np.zeros((self.T, 2))
```

The constructor takes a `VehicleSpecification` object and an optional `CubicSplineCourse`. The key member variables are:

| Variable | Default | Description |
|---|---|---|
| `WHEEL_BASE_M` | from spec | Distance between front and rear axles [m] |
| `T` | 20 | Prediction horizon - number of steps rolled out per sample |
| `K` | 256 | Number of sample trajectories drawn each step |
| `param_lambda` | 50.0 | Temperature $λ$ - controls sharpness of weighting |
| `param_gamma` | $λ(1−α)$ | Control cost scaling; $α=1 → γ=0$ (no control cost) |
| `Sigma` | diag$(σ_δ², σ_a²)$ | Noise covariance matrix for sampling |
| `stage_cost_weight` | [50, 50, 1, 20] | $[w_x, w_y, w_ψ, w_v]$ - stage tracking weights |
| `terminal_cost_weight` | same as stage | $[w_x, w_y, w_ψ, w_v]$ - terminal tracking weights |
| `u_prev` | zeros (T, 2) | Warm-start control sequence `[steer, accel]` per step |
| `target_accel_mps2` | 0.0 | Computed acceleration command [m/s²] |
| `target_steer_rad` | 0.0 | Computed steering angle command [rad] |
| `target_yaw_rate_rps` | 0.0 | Computed yaw rate command [rad/s] |

---

## 6.2 Algorithm Background

### 6.2.1 State and Control Vectors

MPPI operates on a state vector $x_t$ and control vector $u_t$:

$$
x_t
=
\begin{bmatrix}
x \\
y \\
\psi \\
v
\end{bmatrix}
\qquad
\text{(position [m], heading [rad], speed [m/s])}
$$

$$
u_t
=
\begin{bmatrix}
\delta \\
a
\end{bmatrix}
\qquad
\text{(steering angle [rad], acceleration [m/s2])}
$$

The **nominal control sequence** (warm-started from the previous control cycle) is stored as:

$$
U
=
\left\{
u_{\text{prev}}[0],
u_{\text{prev}}[1],
\ldots,
u_{\text{prev}}[T-1]
\right\}
$$

with shape:

$$
(T,\,2)
$$

---

### 6.2.2 Theoretical Foundation - Free Energy and KL Divergence

MPPI is derived from the principle of minimising a **free energy** objective. The theoretical result shows that the optimal control update under a Gaussian prior is equivalent to minimising:

$$
J = E_Q[S(τ)]  +  λ · D_KL(Q ‖ P₀)
$$

where 
- $S(τ)$ is the trajectory cost
- $Q$ is the sampling distribution
-$P₀$ is the uncontrolled prior
- $D_KL$ is the KL divergence measuring how far $Q$ departs from $P_0$.

The closed-form solution to this gives the information-theoretic weight for each sample:
$$
w_k  ∝  exp( −S(k) / λ )
$$

This is exactly the **softmin formula**: lower-cost samples receive exponentially higher weight. The temperature parameter $λ$ controls the sharpness:


- $λ → 0$ :  only the single best sample contributes  (greedy / deterministic)
- $λ → ∞$ :  all samples receive equal weight  (fully random)


---

### 6.2.3 Sampling - Exploitation and Exploration

At each step, K noise sequences are drawn from a zero-mean Gaussian:

```
ε_{k,t}  ~  N(0, Σ)     for k = 1…K,  t = 0…T-1

Σ = diag(σ_steer², σ_accel²)   (diagonal — steer and accel noise are independent)
```

The K samples are split into two groups:

```
Exploitation samples  (first  (1 − param_exploration) × K):
    v_{k,t}  =  clip( u_prev[t]  +  ε_{k,t} )    ← perturb warm start

Exploration samples   (last  param_exploration × K):
    v_{k,t}  =  clip( ε_{k,t} )                   ← pure random, ignores warm start
```

The exploitation samples stay close to the previous solution and refine it. The exploration samples venture further afield and can discover better solutions when the warm start has drifted off-course. The `param_exploration` parameter controls the ratio.

---

### 6.2.4 Trajectory Cost S(k)

Each sample trajectory `k` accumulates cost across all T steps plus a terminal cost:

```
S(k)  =  Σ_{t=0}^{T-1} [ c(x_t^k)  +  γ · u_prev[t]ᵀ Σ⁻¹ v_{k,t} ]   +   ϕ(x_T^k)
```

**Stage tracking cost** `c(x)` — deviation from the reference path:

```
c(x)  =  w_x · (x − x_r)²
        + w_y · (y − y_r)²
        + w_ψ · atan2(sin(ψ − ψ_r), cos(ψ − ψ_r))²
        + w_v · (v − v_r)²
```

The heading error uses `atan2(sin(·), cos(·))` for the same reason as MPC — it wraps the angle difference into `(−π, π]` and avoids discontinuities near ±π.

**Control cost term** `γ · u_prev[t]ᵀ Σ⁻¹ v_{k,t}` — this term is the mathematical signature of the MPPI formulation. It penalises samples that deviate far from the warm-start control `u_prev`. The parameter `γ = λ(1 − α)` controls its strength:

```
param_alpha = 1.0  →  γ = 0   (control cost off — pure tracking)
param_alpha = 0.0  →  γ = λ   (full control cost — conservative)
```

In practice `param_alpha = 0.98` or `1.0` works well for path tracking.

**Terminal cost** `ϕ(x_T)` — same structure as `c(x)` but evaluated only at the last step of each rollout, using `terminal_cost_weight` instead of `stage_cost_weight`.

---

### 6.2.5 Information-Theoretic Weighting

Once all K trajectory costs are computed, the weights are calculated in three steps:

**Step 1** — Subtract the minimum cost for numerical stability (prevents `exp` overflow):

```
ρ  =  min_k  S(k)
```

**Step 2** — Compute unnormalised softmin weights:

```
η̃_k  =  exp( −(S(k) − ρ) / λ )
```

**Step 3** — Normalise so weights sum to 1:

```
η  =  Σ_k  η̃_k
w_k  =  η̃_k / η
```

The result is a proper probability distribution over the K samples. Samples with cost close to `ρ` (the best sample) get weight near `1/K` or higher; samples with cost far above `ρ` get weight near 0.

---

### 6.2.6 Control Update

The weighted perturbation sum is computed across all K samples for each horizon step:

```
w_ε[t]  =  Σ_k  w_k · ε_{k,t}     shape: (T, 2)
```

This is the information-theoretically optimal update direction — a weighted combination of all noise perturbations, pulled toward the samples that worked best.

**Optional smoothing** — a moving-average filter can be applied to `w_ε` along the time axis:

```
w_ε  ←  MovingAverage(w_ε, window)
```

This reduces chatter in the control sequence at the cost of slightly slower response.

**Final update and warm-start shift:**

```
u  =  clip( u_prev  +  w_ε )     ← updated control sequence

apply u[0] → steer0, accel0       ← first action executed this step

warm-start shift for next step:
    u_prev[0..T-2]  ←  u[1..T-1]
    u_prev[T-1]     ←  u[T-1]     ← hold last action
```

---

## 6.3 Private Methods

### 6.3.1 `_get_nearest_waypoint(x, y, update_prev_idx)`

```python
def _get_nearest_waypoint(self, x, y, update_prev_idx=False):
    view = _StateView(x, y, 0.0, 0.0)
    nearest_idx = self.course.search_nearest_point_index(view)
    ref_x   = self.course.point_x_m(nearest_idx)
    ref_y   = self.course.point_y_m(nearest_idx)
    ref_yaw = self.course.point_yaw_rad(nearest_idx)
    ref_v   = self.course.point_speed_mps(nearest_idx)
    if update_prev_idx:
        self.prev_waypoints_idx = nearest_idx
    return ref_x, ref_y, ref_yaw, ref_v
```

This method queries the course for the nearest point to `(x, y)` using a global nearest-neighbour search over all course points. It returns the reference position, heading, and speed at that point. If `update_prev_idx=True`, the found index is stored — this is called once per `update()` step on the vehicle's actual position to anchor the cost lookups. Note that during rollouts, this method is called on the **simulated** position of each sample, not the vehicle's real position.

---

### 6.3.2 `_g(v)` — Control Clipping

```python
def _g(self, v):
    steer = np.clip(v[0], -self.max_steer_abs, self.max_steer_abs)
    accel = np.clip(v[1], -self.max_accel_abs, self.max_accel_abs)
    return np.array([steer, accel])
```

After adding noise to the warm-start control, `_g()` clips the result to the physical limits of the vehicle. This is MPPI's only mechanism for enforcing control bounds — there are no hard constraints as in MPC. Samples that would require `|δ| > δ_max` are simply clipped to the limit before being rolled out.

---

### 6.3.3 `_F(x_t, v_t)` — One-Step Dynamics Rollout

```python
def _F(self, x_t, v_t):
    x, y, yaw, v = x_t[0,0], x_t[1,0], x_t[2,0], x_t[3,0]
    steer, accel = float(v_t[0]), float(v_t[1])
    if abs(v) < 1e-9:
        yaw_rate = 0.0
    else:
        yaw_rate = v / self.WHEEL_BASE_M * tan(steer)
    state_vec = np.array([[x], [y], [yaw], [v]])
    input_vec = np.array([[accel], [yaw_rate]])
    return State.motion_model(state_vec, input_vec, self.delta_t)
```

This method advances the vehicle state by one time step `Δt` using the kinematic bicycle model. It converts the MPPI control `(steer, accel)` into the `(accel, yaw_rate)` format expected by `State.motion_model`:

```
yaw_rate  =  v / L · tan(δ)
```

A guard against near-zero speed (`|v| < 1e-9`) is included to avoid division by zero — the same guard used in the Stanley controller. This function is called `K × T` times per `update()` step — once for each sample, at each horizon step — making it the computational hotspot of the algorithm.

---

### 6.3.4 `_c(x_t)` — Stage Cost

```python
def _c(self, x_t):
    x, y, yaw, v = float(x_t[0,0]), float(x_t[1,0]), float(x_t[2,0]), float(x_t[3,0])
    yaw = (yaw + 2.0 * np.pi) % (2.0 * np.pi)
    ref_x, ref_y, ref_yaw, ref_v = self._get_nearest_waypoint(x, y)
    ref_yaw = (ref_yaw + 2.0 * np.pi) % (2.0 * np.pi)
    yaw_diff = np.arctan2(np.sin(yaw - ref_yaw), np.cos(yaw - ref_yaw))
    cost = (  stage_cost_weight[0] * (x - ref_x)**2
            + stage_cost_weight[1] * (y - ref_y)**2
            + stage_cost_weight[2] * yaw_diff**2
            + stage_cost_weight[3] * (v - ref_v)**2  )
    return cost
```

The stage cost measures how far the **simulated** state `x_t` of sample `k` at step `t` deviates from the nearest reference point on the course. It is evaluated at every step of every rollout.

The yaw is normalised to `[0, 2π)` before computing the heading difference. `atan2(sin(·), cos(·))` then maps the difference back to `(−π, π]`, giving the shortest-path angle error regardless of which quadrant the vehicle is in.

The nearest reference point is found by a **global search** — unlike the MPC controller which uses an arc-length scaled forward search, MPPI calls `_get_nearest_waypoint()` with the simulated position at each rollout step. This works well because MPPI's rollouts are typically short and stay near the course.

---

### 6.3.5 `_phi(x_T)` — Terminal Cost

```python
def _phi(self, x_T):
    # Same structure as _c(), uses terminal_cost_weight
    cost = (  terminal_cost_weight[0] * (x - ref_x)**2
            + terminal_cost_weight[1] * (y - ref_y)**2
            + terminal_cost_weight[2] * yaw_diff**2
            + terminal_cost_weight[3] * (v - ref_v)**2  )
    return cost
```

The terminal cost is evaluated only at the **last state** `x_T^k` of each sample rollout. It uses `terminal_cost_weight` instead of `stage_cost_weight`. By default the weights are the same, but setting the terminal weights higher encourages samples to end up in a good state at the end of the horizon — analogous to the terminal cost in MPC.

---

### 6.3.6 `_calc_epsilon()`

```python
def _calc_epsilon(self):
    mu = np.zeros(2)
    epsilon = np.random.multivariate_normal(mu, self.Sigma, (self.K, self.T))
    return epsilon
```

Samples the entire noise matrix $ε ∈ ℝ^{K × T × 2}$ in a single vectorised call using numpy's `multivariate_normal`. The shape $(K, T, 2)$ ` means: K samples, each of T steps, each with 2-dimensional noise `
$[\epsilon_{steer}, \epsilon_{accel}]$.

The covariance matrix is:

$$
Σ = diag(σ_steer², σ_accel²)  =  [[σ_steer²,    0    ],
                                    [   0,      σ_accel²]]
$$

The off-diagonal zeros mean steering and acceleration noise are drawn independently. A single `multivariate_normal` call is used instead of two separate `normal` calls to keep the code consistent with the MPPI formulation, which always refers to a single noise vector $\epsilon_t ∈ ℝ²$.

---

### 6.3.7 `_compute_weights(S)`

```python
def _compute_weights(self, S):
    rho = S.min()
    eta = np.sum(np.exp((-1.0 / self.param_lambda) * (S - rho)))
    w   = (1.0 / eta) * np.exp((-1.0 / self.param_lambda) * (S - rho))
    return w
```

Implements the information-theoretic weighting formula in three lines. The subtraction of $ρ = min(S)$ before the exponential is not a mathematical change, it is a **numerical stability trick**. Without it, $exp(−S/λ)$ would underflow to $0$ for all samples when $S$ values are large, making the weights undefined. Subtracting $ρ$ ensures the best sample always contributes $exp(0) = 1.0$ to the numerator.

The output $w$ is a $(K,)$ array that sums to $1.0$, acting as a proper probability distribution over the $K$ samples.

---

### 6.3.8 `_moving_average_filter(xx, window_size)`

```python
def _moving_average_filter(self, xx, window_size):
    b = np.ones(window_size) / window_size
    xx_mean = np.zeros_like(xx)
    for d in range(xx.shape[1]):
        xx_mean[:, d] = np.convolve(xx[:, d], b, mode="same")
        n_conv = math.ceil(window_size / 2)
        xx_mean[0, d] *= window_size / n_conv
        for i in range(1, n_conv):
            xx_mean[i, d]  *= window_size / (i + n_conv)
            xx_mean[-i, d] *= window_size / (i + n_conv - (window_size % 2))
    return xx_mean
```

Applies a moving-average filter to each column of the $(T, 2)$ weighted perturbation array $w_ε$. The filter smooths the control sequence along the time dimension, reducing high-frequency chatter that can occur when different samples pull the update in opposite directions at adjacent timesteps.

The edge correction loop rescales the beginning and end of the filtered signal where the convolution window extends beyond the available data - `numpy`'s `mode="same"` convolves with zero-padding at the edges, which effectively down-weights the first and last few values. The correction counteracts this by scaling them back up proportionally.

---

## 6.4 Public Methods

### 6.4.1 `update`

```python
def update(self, state, time_s):
    if not self.course: return

    x0 = np.array([[state.get_x_m()],
                   [state.get_y_m()],
                   [state.get_yaw_rad()],
                   [state.get_speed_mps()]])

    self._get_nearest_waypoint(x0[0,0], x0[1,0], update_prev_idx=True)
    u       = self.u_prev.copy()
    epsilon = self._calc_epsilon()

    # Build v_{k,t} — perturbed controls for each sample
    n_exploit = int((1.0 - self.param_exploration) * self.K)
    for k in range(self.K):
        for t in range(self.T):
            v[k,t] = self._g(u[t] + epsilon[k,t]) if k < n_exploit \
                     else self._g(epsilon[k,t])

    # Rollout and cost accumulation
    Sigma_inv = np.linalg.inv(self.Sigma)
    for k in range(self.K):
        x = x0.copy()
        for t in range(self.T):
            S[k] += self._c(x) + self.param_gamma * (u[t].T @ Sigma_inv @ v[k,t])
            x     = self._F(x, v[k,t])
        S[k] += self._phi(x)

    w         = self._compute_weights(S)
    w_epsilon = sum_k( w[k] * epsilon[k] )      # shape (T, 2)
    w_epsilon = self._moving_average_filter(w_epsilon, window)
    u         = clip( u_prev + w_epsilon )

    self.target_steer_rad    = u[0, 0]
    self.target_accel_mps2   = u[0, 1]
    self.target_yaw_rate_rps = v0 / L * tan(steer0)

    self.u_prev[:-1] = u[1:]
    self.u_prev[-1]  = u[-1]
```

This is the main entry point called every simulation frame by `FourWheelsVehicle`. It orchestrates all private methods in order:

```
update()
  1.  Build x0 (4×1) from state getters
  2.  Anchor nearest waypoint index (update_prev_idx=True)
  3.  Copy u_prev as warm-start nominal sequence u
  4.  Sample ε ∈ ℝ^{K×T×2} via _calc_epsilon()
  5.  Build v_{k,t} — exploitation or exploration + clip via _g()
  6.  For each sample k:
        For each step t:
          S[k] += _c(x)  +  γ · u[t]ᵀ Σ⁻¹ v[k,t]
          x     = _F(x, v[k,t])
        S[k] += _phi(x)
  7.  w  = _compute_weights(S)
  8.  w_ε[t] = Σ_k w[k] · ε[k,t]   for all t
  9.  Optional: _moving_average_filter(w_ε)
  10. u  = clip(u_prev + w_ε)
  11. target_steer, target_accel ← u[0]
  12. target_yaw_rate = v / L · tan(steer)
  13. Store optimal and sampled trajectories for draw()
  14. Shift warm start: u_prev[0..T-2] ← u[1..T-1]
```

If no course is set, the method returns early without computing anything.

---

### 6.4.2 Getter Methods

```python
def get_target_accel_mps2(self):
    return self.target_accel_mps2

def get_target_yaw_rate_rps(self):
    return self.target_yaw_rate_rps

def get_target_steer_rad(self):
    return self.target_steer_rad
```

These three getter methods expose the computed control outputs. They are called by `FourWheelsVehicle` after each `update()` to apply the commands to the vehicle's state. The interface is identical to `StanleyController` and `MpcController`.

---

### 6.4.3 `draw`

```python
def draw(self, axes, elems):
    if self.visualize_sampled_trajs and self.sampled_trajectories:
        for (x_list, y_list), w in zip(self.sampled_trajectories, self.weights):
            alpha = 0.06 + 0.12 * min(1.0, float(w) * self.K)
            (line,) = axes.plot(x_list, y_list, "b-", linewidth=0.35, alpha=alpha)
            elems.append(line)

    if self.visualize_optimal_traj and self.optimal_trajectory:
        x_list, y_list = self.optimal_trajectory
        (line,) = axes.plot(x_list, y_list, color=self.DRAW_COLOR,
                            linewidth=2.0, alpha=0.9, label="MPPI trajectory")
        elems.append(line)
```

Unlike the Stanley controller where `draw()` is an empty `pass`, and unlike the MPC controller which draws one line, MPPI draws **two layers of visual information**:

**Sampled trajectories** - all $K$ rollouts are drawn as thin blue lines. Each line's transparency ($\alpha$) is scaled by the sample's weight:

$$
alpha  =  0.06  +  0.12 · min(1.0, w_k · K)
$$

A sample with average weight ($w_k = 1/K$) gets $\alpha ≈ 0.18$. A sample with $5×$ average weight gets $alpha = 0.66$. This makes the most-influential samples visually prominent and the low-weight samples nearly invisible - you can literally see which trajectories the controller is paying attention to(darker).

**Optimal trajectory** - a single solid line in the constructor colour (default green) showing the optimal control sequence $u$ rolled out from the current state. This is the trajectory the controller has committed to executing, not just the best single sample - it is the weighted-average result after all K samples are combined.

---

## 6.5 Comparison with MPC

| Aspect | MPC | MPPI |
|----------|----------|----------|
| Optimization Method | Deterministic nonlinear optimization | Sampling-based stochastic optimization |
| Error Used | Heading + position + speed | Heading + position + speed |
| Prediction Horizon | N-step deterministic trajectory | T-step horizon with K sampled rollouts |
| Constraints | Hard constraints enforced by solver | Soft constraints via clipping and cost penalties |
| Solver | IPOPT / SQP / QP solver | No explicit solver; weighted rollout averaging |
| Compute Cost | ~10–50 ms (CPU, IPOPT) | ~1–5 ms (CPU), <1 ms (GPU) |
| Local Minima | Can get trapped in local minima | Better exploration through stochastic sampling |
| Tuning Parameters | Cost weights and horizon length | $\lambda$, $K$, $\sigma$, $\alpha$, horizon length |
| Dynamics Requirement | Requires differentiable model | Can use arbitrary black-box dynamics |
| Parallelization | Limited | Highly parallelizable (GPU-friendly) |
| Visualization | Single predicted optimal trajectory | All sampled rollouts plus optimal trajectory |
| Robustness to Model Errors | Sensitive to model mismatch | More robust due to sampling-based exploration |
| Real-Time Performance | Depends on solver convergence | Fixed computational budget via sample count |

---

**Author**: Mohit Kumar

# Notation and Units

This page defines the symbols and units used in the whole book. Each chapter follows it, and the [writing guide](WRITING_GUIDE.md) refers to it. When a chapter needs a symbol that is not here, add it to this page in the same pull request.

## General rules

- Scalars are italic ($v$, $\delta$), vectors are bold lowercase ($\mathbf{x}$, $\mathbf{u}$), and matrices are uppercase ($P$, $F$).
- The time step index is a subscript: $\mathbf{x}_t$ is the state at step $t$, and $\Delta t$ is the time between two steps.
- An estimate has a hat ($\hat{\mathbf{x}}_t$), and a prediction before the observation is used has a bar ($\bar{\mathbf{x}}_t$).
- $^\top$ is the transpose, and $I$ is the identity matrix.
- Angles in equations are in radians, and angle differences are wrapped into $[-\pi, \pi)$.

## Vehicle state and inputs

| Symbol | Meaning | Unit | In the code |
|---|---|---|---|
| $x, y$ | Position of the vehicle (rear axle center) in the global frame | m | `x_m`, `y_m` |
| $\theta$ | Yaw (heading) angle, counterclockwise from the global x axis | rad | `yaw_rad` |
| $v$ | Speed | m/s | `speed_mps` |
| $\mathbf{x}_t = [x, y, \theta, v]^\top$ | State vector | - | `State`, state arrays (4x1) |
| $a$ | Acceleration | m/s² | `accel_mps2` |
| $\omega$ | Yaw rate | rad/s | `yaw_rate_rps` |
| $\delta$ | Steering angle of the front wheels | rad | `steer_rad` |
| $\mathbf{u}_t = [a, \omega]^\top$ | Input vector of the motion model | - | input arrays (2x1) |
| $L$ | Wheelbase, the distance between the front and rear axles | m | `wheel_base_m` |
| $\Delta t$ | Time step | s | `time_s` |

## Estimation

| Symbol | Meaning |
|---|---|
| $\mathbf{z}_t$ | Observation, for example a GNSS position $[x, y]^\top$ |
| $f(\mathbf{x}, \mathbf{u})$ | Motion model: $\mathbf{x}_t = f(\mathbf{x}_{t-1}, \mathbf{u}_t)$ |
| $h(\mathbf{x})$ | Observation model: $\mathbf{z}_t = h(\mathbf{x}_t)$ |
| $\hat{\mathbf{x}}_t$, $P_t$ | Estimated state and its covariance |
| $\bar{\mathbf{x}}_t$, $\bar{P}_t$ | Predicted state and its covariance, before the observation is used |
| $F = \partial f / \partial \mathbf{x}$ | Jacobian of the motion model with respect to the state |
| $G = \partial f / \partial \mathbf{u}$ | Jacobian of the motion model with respect to the input |
| $H = \partial h / \partial \mathbf{x}$ | Jacobian of the observation model |
| $Q$ | Covariance of the input (process) noise |
| $R$ | Covariance of the observation noise |
| $S$ | Covariance of the innovation $\mathbf{z}_t - h(\bar{\mathbf{x}}_t)$ |
| $K$ | Kalman gain |

With these symbols, the EKF prediction is $\bar{P}_t = F P_{t-1} F^\top + G Q G^\top$, and the innovation covariance is $S = H \bar{P}_t H^\top + R$.

## Path tracking

| Symbol | Meaning | Unit |
|---|---|---|
| $e$ | Lateral error: signed distance from the reference course to the tracked point, positive when the point is on the left of the course | m |
| $\theta_e$ | Heading error: $\theta - \theta_r$, the vehicle's yaw minus the course's yaw at the target point, wrapped into $[-\pi, \pi)$ | rad |
| $\theta_r$, $v_r$ | Yaw and speed of the reference course at the target point | rad, m/s |
| $\kappa$ | Curvature of the course | 1/m |
| $k$ | Gain of a controller (each chapter states which one) | - |

## Frames

| Frame | Origin and axes |
|---|---|
| Global $\{G\}$ | Fixed to the world. x to the right and y up in the plots, angles counterclockwise from x |
| Vehicle $\{V\}$ | Rear axle center. x forward, y to the left |
| Sensor $\{S\}$ | Mounting point of the sensor. x along the sensor's forward direction |

A point $\mathbf{p}$ written in frame $\{A\}$ is ${}^{A}\mathbf{p}$. The 2D rotation matrix is written $\mathrm{Rot}(\theta)$, not $R$, so it is not mixed up with the observation noise covariance:

$$
\mathrm{Rot}(\theta) =
\begin{bmatrix}
\cos\theta & -\sin\theta \\
\sin\theta & \cos\theta
\end{bmatrix},
\qquad
{}^{G}\mathbf{p} = \mathrm{Rot}(\theta)\, {}^{V}\mathbf{p} + [x, y]^\top
$$

## Units in the code

Equations use SI units. In the code, the unit is part of the variable name, so a reader can check the units of an equation from the code:

| Suffix | Unit | Example |
|---|---|---|
| `_m` | m | `wheel_base_m` |
| `_rad` | rad | `yaw_rad` |
| `_deg` | degree (only for parameters that people set by hand, converted to radians before use) | `yaw_rate_noise` in degrees |
| `_mps` | m/s | `speed_mps` |
| `_mps2` | m/s² | `accel_mps2` |
| `_rps` | rad/s | `yaw_rate_rps` |
| `_s` | s (`_sec` in older code) | `time_s` |

## Mapping to the main references

The references use other symbols for the same quantities. The tables below map them, so readers can follow a derivation in a reference and come back to the book.

**Probabilistic Robotics** (S. Thrun, W. Burgard and D. Fox, MIT Press, 2005), EKF localization:

| This book | Probabilistic Robotics | Note |
|---|---|---|
| $\hat{\mathbf{x}}_t$, $P_t$ | $\mu_t$, $\Sigma_t$ | Mean and covariance of the belief |
| $\bar{\mathbf{x}}_t$, $\bar{P}_t$ | $\bar{\mu}_t$, $\bar{\Sigma}_t$ | |
| $F$ | $G_t$ | Jacobian with respect to the state |
| $G$ | $V_t$ | Jacobian with respect to the control |
| $Q$ | $M_t$ | Control (input) noise covariance |
| $G Q G^\top$ | $R_t$ | Motion noise covariance in the state space |
| $R$ | $Q_t$ | Measurement noise covariance |

This book keeps $Q$ for the input noise and $R$ for the observation noise, as in the code and in most Kalman filter tutorials. Probabilistic Robotics uses $R_t$ and $Q_t$ the other way around. The code is not renamed.

**Vehicle Dynamics and Control** (R. Rajamani, 2nd ed., Springer, 2012), kinematic bicycle model and lateral control:

| This book | Rajamani | Note |
|---|---|---|
| $\theta$ | $\psi$ | Yaw angle |
| $L$ | $l_f + l_r$ | Wheelbase |
| $e$, $\theta_e$ | $e_1$, $e_2$ | Lateral and heading errors |

**Automatic Steering Methods for Autonomous Automobile Path Tracking** (J. M. Snider, CMU-RI-TR-09-08, 2009), Stanley controller:

| This book | Snider | Note |
|---|---|---|
| $e$ | $e_{fa}$ | Lateral error at the front axle |
| $\theta_e$ | $\theta_e$ | Heading error. The sign convention differs, see the [Stanley chapter](4_path_tracking/4_1_stanley_controller.md) |

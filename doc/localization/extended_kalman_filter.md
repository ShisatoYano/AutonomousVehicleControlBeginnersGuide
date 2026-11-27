# 6.2 Extended Kalman filter

The extended Kalman filter (EKF) for vehicle localization explained, with its equations and a Python simulation. The EKF estimates the vehicle's pose by combining two imperfect sources: the inputs integrated with the motion model, which drift, and GNSS positions, which are noisy and have no heading. After this section, you can derive the prediction and the update of the EKF from the motion model of [Chapter 3](../vehicle_model/index.md), and explain how a sensor that only measures the position also corrects the heading.

## Problem and intuition

The vehicle has two ways to know where it is, and neither is good enough on its own:

- **Dead reckoning.** Integrate the acceleration and the yaw rate with the motion model, starting from a known pose. Each step is accurate, but the inputs are noisy, and every error is carried into the next step. A small heading error turns into a growing position error, so the estimate drifts away without limit.
- **GNSS.** A GNSS receiver measures the position directly, so its error doesn't grow. But each measurement has an error of a meter or more, and it measures no heading and no speed.

The two fail in opposite ways: dead reckoning is smooth but drifts, and GNSS doesn't drift but jumps. The Kalman filter [1] combines them by weighting each with its uncertainty. It keeps a Gaussian belief about the state, a mean and a covariance, and repeats two steps: **predict** the state with the motion model, which makes the belief less certain, then **update** it with the GNSS position, which makes it more certain. When the prediction is uncertain, the update follows the GNSS position; when the GNSS position is the more uncertain one, it follows the prediction.

The motion model of a vehicle is nonlinear in the heading, and the Kalman filter needs linear models. The **extended** Kalman filter linearizes the model around the current estimate at every step.

```{figure} figures/ekf_run.svg
:alt: A path of about 85 m that weaves left and right. The GNSS positions are scattered around it by about a meter. Dead reckoning starts on the path and drifts away from it, ending about 8 m off. The EKF estimate stays on the path. A zoomed box shows a small covariance ellipse of the EKF on the path, among the scattered GNSS positions.

One run of 30 s with the motion model and the filter of the simulation, the inputs noisy with the $Q$ of the code and GNSS with a standard deviation of 1 m. The root mean square of the position error over the run is 5.4 m for dead reckoning, 1.4 m for the GNSS positions and 0.38 m for the EKF. The ellipses of the EKF's covariance are about 1 m across, so one of them is zoomed in.
```

**Run it** from the repository root: `python src/simulations/localization/extended_kalman_filter_localization/extended_kalman_filter_localization.py` ([code on GitHub](/src/simulations/localization/extended_kalman_filter_localization))

## Theory

The symbols follow [Notation and Units](../NOTATION.md). The state is the pose and the speed, the input is the acceleration and the yaw rate, and the observation is the GNSS position:

$$
\mathbf{x}_t = [x, y, \theta, v]^\top, \qquad \mathbf{u}_t = [a, \omega]^\top, \qquad \mathbf{z}_t = [x, y]^\top \tag{1}
$$

with $x, y$ in m, $\theta$ in rad, $v$ in m/s, $a$ in m/s² and $\omega$ in rad/s. The time step is $\Delta t$ in s.

### The models with noise

The motion model $f$ is the discrete-time model (4) of [Chapter 3](../vehicle_model/index.md#33-from-continuous-to-discrete-time), written from step $t-1$ to step $t$. The vehicle moves the distance $d = v\Delta t + \frac{1}{2} a \Delta t^2$ along its heading:

\begin{equation*}
f(\mathbf{x}, \mathbf{u}) =
\begin{bmatrix}
x + d\cos\theta \\
y + d\sin\theta \\
\theta + \omega \Delta t \\
v + a \Delta t
\end{bmatrix} \tag{2}
\end{equation*}

The inputs that the vehicle measures are noisy. The noise $\boldsymbol{\varepsilon}_t$ is added to the input, with a Gaussian distribution of mean zero and covariance $Q$:

$$
\mathbf{x}_t = f(\mathbf{x}_{t-1}, \mathbf{u}_t + \boldsymbol{\varepsilon}_t), \qquad \boldsymbol{\varepsilon}_t \sim \mathcal{N}(\mathbf{0}, Q), \qquad Q = \mathrm{diag}(\sigma_a^2, \sigma_\omega^2) \tag{3}
$$

GNSS observes the position with the noise $\boldsymbol{\eta}_t$ of covariance $R$:

\begin{equation*}
\mathbf{z}_t = H \mathbf{x}_t + \boldsymbol{\eta}_t, \qquad
H = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \end{bmatrix}, \qquad
\boldsymbol{\eta}_t \sim \mathcal{N}(\mathbf{0}, R), \qquad R = \mathrm{diag}(\sigma_x^2, \sigma_y^2) \tag{4}
\end{equation*}

The observation model $h(\mathbf{x}) = H\mathbf{x}$ is linear, so only the motion model needs a linearization. In the simulation, $\sigma_a = 0.2$ m/s², $\sigma_\omega = 10$ deg/s (0.17 rad/s) and $\sigma_x = \sigma_y = 1$ m.

### The Kalman gain in one dimension

The whole filter rests on one idea, which is easiest to see with one number. Say a prediction gives the position $\bar{x}$ with the variance $\bar{\sigma}^2$, and a sensor measures $z$ with the variance $\sigma_z^2$. The best combination of the two, the one with the smallest variance, is

$$
\hat{x} = \bar{x} + k\,(z - \bar{x}), \qquad k = \frac{\bar{\sigma}^2}{\bar{\sigma}^2 + \sigma_z^2}, \qquad \hat{\sigma}^2 = (1 - k)\,\bar{\sigma}^2 \tag{5}
$$

The gain $k$ is between 0 and 1, and $\hat{x} = (1 - k)\,\bar{x} + k\,z$ is a weighted mean: an uncertain prediction (large $\bar{\sigma}^2$) gives $k$ close to 1 and follows the measurement, and a noisy sensor (large $\sigma_z^2$) gives $k$ close to 0 and keeps the prediction. The variance $\hat{\sigma}^2$ is smaller than both $\bar{\sigma}^2$ and $\sigma_z^2$: two uncertain sources together are more certain than either. Section 6.1, the Kalman filter in one dimension (planned, see the [chapter's list](index.md)), will derive (5). The EKF does the same with vectors and matrices.

### Prediction

The estimate of the last step is the mean $\hat{\mathbf{x}}_{t-1}$ with the covariance $P_{t-1}$. The predicted mean moves the estimate with the motion model and the measured input:

$$
\bar{\mathbf{x}}_t = f(\hat{\mathbf{x}}_{t-1}, \mathbf{u}_t) \tag{6}
$$

For the covariance, $f$ is replaced by its first-order Taylor expansion around $\hat{\mathbf{x}}_{t-1}$ and $\mathbf{u}_t$:

$$
\mathbf{x}_t \approx f(\hat{\mathbf{x}}_{t-1}, \mathbf{u}_t) + F\,(\mathbf{x}_{t-1} - \hat{\mathbf{x}}_{t-1}) + G\,\boldsymbol{\varepsilon}_t, \qquad F = \frac{\partial f}{\partial \mathbf{x}}, \quad G = \frac{\partial f}{\partial \mathbf{u}} \tag{7}
$$

The error of the last estimate, $\mathbf{x}_{t-1} - \hat{\mathbf{x}}_{t-1}$, has the covariance $P_{t-1}$, and the input noise $\boldsymbol{\varepsilon}_t$ has $Q$ and doesn't depend on it. A linear map $A\mathbf{e}$ of an error with covariance $\Sigma$ has the covariance $A \Sigma A^\top$, and the covariances of independent errors add up, so

$$
\bar{P}_t = F P_{t-1} F^\top + G Q G^\top \tag{8}
$$

The first term carries the old uncertainty through the motion, and the second term adds the new uncertainty of the inputs. $G$ also converts $Q$, which is in the units of the inputs, into the units of the state.

The Jacobians come from differentiating (2) row by row. Apart from the ones on the diagonal, only the first two rows depend on $\theta$ and $v$, through $d\cos\theta$ and $d\sin\theta$ with $\partial d / \partial v = \Delta t$:

\begin{equation*}
F =
\begin{bmatrix}
1 & 0 & -d\sin\theta & \Delta t\cos\theta \\
0 & 1 & d\cos\theta & \Delta t\sin\theta \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}, \qquad
G =
\begin{bmatrix}
\tfrac{1}{2}\Delta t^2\cos\theta & 0 \\
\tfrac{1}{2}\Delta t^2\sin\theta & 0 \\
0 & \Delta t \\
\Delta t & 0
\end{bmatrix} \tag{9}
\end{equation*}

with $d = v\Delta t + \frac{1}{2} a \Delta t^2$. The column of $\theta$ in $F$ is where the model is nonlinear: a heading error $\Delta\theta$ moves the predicted position sideways by about $d\,\Delta\theta$. $f$ is linear in the input, so $G$ is the matrix $B(\theta)$ of Chapter 3.

### Update

With $h(\mathbf{x}) = H\mathbf{x}$, the update is the one of the linear Kalman filter. The **innovation** $\mathbf{z}_t - H\bar{\mathbf{x}}_t$ is the difference between the GNSS position and the predicted position. Its covariance adds the uncertainty of the prediction and of GNSS:

$$
S = H \bar{P}_t H^\top + R \tag{10}
$$

The Kalman gain and the new estimate are

$$
K = \bar{P}_t H^\top S^{-1} \tag{11}
$$

$$
\hat{\mathbf{x}}_t = \bar{\mathbf{x}}_t + K\,(\mathbf{z}_t - H\bar{\mathbf{x}}_t) \tag{12}
$$

$$
P_t = (I - K H)\,\bar{P}_t \tag{13}
$$

The code computes (13) as $\bar{P}_t - K S K^\top$, which is the same matrix because $K S K^\top = \bar{P}_t H^\top S^{-1} S K^\top = \bar{P}_t H^\top K^\top = K H \bar{P}_t$ ($\bar{P}_t$ and $S$ are symmetric).

### Reading the gain

$K$ is a $4 \times 2$ matrix: one row for each element of the state, one column for each coordinate of GNSS. Write the blocks of $\bar{P}_t$ as $\bar{P}_{pp}$ for the position $[x, y]$ and $\bar{P}_{ep}$ for the covariance between the other elements $[\theta, v]$ and the position. Then $H\bar{P}_t H^\top = \bar{P}_{pp}$ and

\begin{equation*}
K = \begin{bmatrix} \bar{P}_{pp}\,(\bar{P}_{pp} + R)^{-1} \\ \bar{P}_{ep}\,(\bar{P}_{pp} + R)^{-1} \end{bmatrix} \tag{14}
\end{equation*}

The upper block is (5) with matrices: the predicted position and the GNSS position are averaged with weights from $\bar{P}_{pp}$ and $R$. The lower block corrects the heading and the speed, which GNSS doesn't measure, through their covariance with the position. That covariance comes from $F$ in (8): if the heading is off, the vehicle drifts sideways, so a GNSS position on one side of the prediction is evidence that the heading points that way.

## Algorithm

```text
Given the last estimate x̂_{t-1}, P_{t-1}, the input u_t and the GNSS position z_t:

Predict
1. Compute F and G with (9) at the last estimate and the input
2. Predict the mean:        x̄_t = f(x̂_{t-1}, u_t)              (6)
3. Predict the covariance:  P̄_t = F P_{t-1} Fᵀ + G Q Gᵀ        (8)

Update
4. Innovation covariance:   S = H P̄_t Hᵀ + R                   (10)
5. Kalman gain:             K = P̄_t Hᵀ S⁻¹                     (11)
6. New mean:                x̂_t = x̄_t + K (z_t - H x̄_t)       (12)
7. New covariance:          P_t = (I - K H) P̄_t                (13)
```

Without a GNSS position in a step, the filter skips the update and keeps $\hat{\mathbf{x}}_t = \bar{\mathbf{x}}_t$ and $P_t = \bar{P}_t$: it does dead reckoning, and its covariance grows until the next position arrives. The simulation has a GNSS position at every step, so the code always updates.

## Limits

- **The linearization is only right for small uncertainty.** (7) treats $f$ as linear over the spread of the belief. With a large heading uncertainty, the poses that the belief allows end on an arc, not in an ellipse:

  ```{figure} figures/ekf_linearization.svg
  :alt: Two thousand poses start near the origin, driving along x at 5 m/s with headings spread by 0.4 rad. After 2 s they lie on an arc around the start, curved toward it. The 95% ellipse of the EKF is a tall ellipse centered at x 10 m, which misses the ends of the arc. The mean of the poses is at about x 9.2 m, behind the EKF's mean at 10 m.

  2000 poses drawn from a Gaussian with standard deviations of 0.3 m in $x$ and $y$, 0.4 rad (23 deg) in $\theta$ and 0.3 m/s in $v$, moved for 2 s by the motion model (2) without input. The EKF predicts their mean with (6) and their covariance with (8). 21.7% of the poses are outside the EKF's 95% ellipse, which a Gaussian would leave 5% outside, and their mean is 0.76 m behind the EKF's mean.
  ```

  The EKF works when the time step is short and the updates keep the heading uncertainty small, as GNSS does at every step of the simulation. The code starts with $P_0 = I$, a heading uncertainty of 1 rad, and relies on the first updates to shrink it. The [unscented Kalman filter](index.md) (6.3) moves sample points through $f$ instead of linearizing it, and follows the arc better.
- **The heading is only observable while the vehicle moves.** GNSS measures the position, and the heading is corrected through its covariance with the position in (14). That covariance is built by the term $d$ of $F$ in (9), which is zero when the vehicle stands still. A vehicle at rest can't learn its heading from GNSS, and a vehicle that starts at rest learns it only after it has driven some distance.
- **Gaussian noise.** The filter assumes that every noise is Gaussian with a known covariance. A real GNSS receiver sometimes gives a position that is many meters off, for example from a signal reflected by a building, and (12) moves the estimate toward it as far as toward any other measurement. Real systems check the innovation against $S$ and reject outliers.
- **Choosing $Q$ and $R$.** $R$ comes from the accuracy of the GNSS receiver, and $Q$ from the accuracy of the acceleration and yaw rate sensors. In the simulation, the filter knows them exactly, because it adds the noise itself. On a real vehicle they are tuned: a $Q$ that is too small makes the filter trust its prediction and react slowly to GNSS, and an $R$ that is too small makes the estimate jump with every GNSS position.
- **What the code does differently.**
  - `ExtendedKalmanFilterLocalizer` evaluates $F$ and $G$ at the predicted state $\bar{\mathbf{x}}_t$, not at the last estimate $\hat{\mathbf{x}}_{t-1}$ as in (7) and in [2]. The two differ by one step of motion; with the short time step the result is almost the same.
  - The filter adds the input noise $\boldsymbol{\varepsilon}_t$ to the input itself, before (6), to simulate noisy sensors. So the mean of the prediction uses a noisy input, as dead reckoning would.
  - The simulation has no true pose separate from the estimate: the vehicle's state is replaced by the estimate at every step, and GNSS measures the vehicle's state with noise. The first figure of this section keeps a true path apart, to compare the estimate with it.
  - The filter starts at $\hat{\mathbf{x}}_0 = \mathbf{0}$ with $P_0 = I$.

## Check your understanding

1. A vehicle stands still with a well known position, but its heading is unknown: a large variance in $P$, and no covariance with the position. GNSS positions arrive at every step, the inputs are zero, and the estimated speed stays zero. Does the heading estimate improve?

   <details><summary>Answer</summary>

   No. With $v = 0$ and $a = 0$, the distance $d$ in (9) is zero, so $F$ doesn't create any covariance between the heading and the position, and neither does $G Q G^\top$, where the yaw rate noise only reaches the heading. The heading row of $\bar{P}_{ep}$ in (14) stays zero, so the heading row of $K$ is zero, and (12) never changes the heading. The speed row is not zero: the column of $v$ in $F$ and the acceleration noise relate the speed to the position, so GNSS corrects the speed. The heading only becomes observable once the vehicle moves. In a real run, the GNSS noise moves the estimated speed a little away from zero, which makes $d$ nonzero and lets the heading be corrected slowly.

   </details>

2. The GNSS receiver is replaced by one that is ten times less accurate ($\sigma_x$ and $\sigma_y$ ten times larger). What happens to the gain and to the estimate?

   <details><summary>Answer</summary>

   $R$ becomes 100 times larger, so $S$ in (10) grows and $K$ in (11) shrinks: as in (5), a noisy sensor gets a small weight. The estimate follows the prediction more closely, so it is smoother but drifts more between corrections, and its covariance $P_t$ stays larger.

   </details>

3. Why does (8) add $G Q G^\top$ to the predicted covariance, and not $Q$ itself?

   <details><summary>Answer</summary>

   $Q$ is the covariance of the input noise, a $2 \times 2$ matrix in (m/s²)² and (rad/s)², while $\bar{P}_t$ is the $4 \times 4$ covariance of the state in m, rad and m/s. By (7), the input noise moves the state by $G\boldsymbol{\varepsilon}_t$, so its covariance in the state is $G Q G^\top$. For example, a yaw rate error changes the heading by $\Delta t$ times as much, and the position only in the following steps.

   </details>

```{note}
This section has its theory. The code walkthrough, experiments and exercises are added after the code roadmap's Phase 1 ([#131](https://github.com/ShisatoYano/AutonomousVehicleControlBeginnersGuide/issues/131)), when the interfaces they show are stable.
```

## References

1. R. E. Kalman, "A New Approach to Linear Filtering and Prediction Problems", Journal of Basic Engineering, vol. 82, no. 1, pp. 35–45, 1960. The paper that introduced the Kalman filter. [DOI](https://doi.org/10.1115/1.3662552)
2. S. Thrun, W. Burgard and D. Fox, "Probabilistic Robotics", MIT Press, 2005. Chapter 3, the Kalman filter and the extended Kalman filter, and Chapter 7, EKF localization. It writes the symbols of this section differently, with $Q$ and $R$ the other way around; see the table in [Notation and Units](../NOTATION.md#mapping-to-the-main-references).

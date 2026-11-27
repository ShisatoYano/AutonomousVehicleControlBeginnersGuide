"""
ekf_linearization.py

Figure of the limit of the linearization in the extended Kalman filter: poses
drawn from a Gaussian with a large yaw uncertainty, moved by the motion model,
form a curved banana, while the EKF prediction P = F P F^T gives an ellipse

Author: Shisato Yano
"""

import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

import figure_style as style

sys.path.append(str(Path(__file__).absolute().parents[3] / "src" / "components" / "state"))
from state import State

DT = 0.1           # time step [s]
STEPS = 20         # steps without input, 2 s in total
SAMPLES = 2000
CHI2_95 = 5.991    # squared Mahalanobis distance that holds 95% of a 2D Gaussian
MEAN = np.array([[0.0], [0.0], [0.0], [5.0]])  # [x, y, theta, v]: at the origin, 5 m/s along x
STD = np.array([0.3, 0.3, 0.4, 0.3])           # yaw uncertainty 0.4 rad, about 23 deg
U = np.zeros((2, 1))

rng = np.random.default_rng(0)
points = MEAN + STD.reshape(4, 1) * rng.standard_normal((4, SAMPLES))
mean, cov = MEAN.copy(), np.diag(STD ** 2)
for _ in range(STEPS):
    points = np.hstack([State.motion_model(points[:, [i]], U, DT) for i in range(SAMPLES)])
    # EKF prediction without input noise: the Jacobian F of section 6.2 at the mean
    th, v = mean[2, 0], mean[3, 0]
    F = np.array([[1, 0, -v * DT * np.sin(th), DT * np.cos(th)],
                  [0, 1, v * DT * np.cos(th), DT * np.sin(th)],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    mean = State.motion_model(mean, U, DT)
    cov = F @ cov @ F.T

# Share of the moved points outside the 95% ellipse of the EKF, quoted in the caption
diff = points[:2] - mean[:2]
d2 = np.sum(diff * np.linalg.solve(cov[:2, :2], diff), axis=0)
OUTSIDE_95 = float(np.mean(d2 > CHI2_95))
SAMPLE_MEAN_X = float(points[0].mean())   # the points' mean falls behind the EKF mean
EKF_MEAN_X = float(mean[0, 0])

eig_val, eig_vec = np.linalg.eigh(cov[:2, :2])
t = np.linspace(0.0, 2.0 * np.pi, 200)
circle = np.sqrt(CHI2_95) * np.vstack([np.cos(t), np.sin(t)])
ellipse = mean[:2] + eig_vec @ (np.sqrt(eig_val).reshape(2, 1) * circle)

fig = plt.figure(figsize=(6.4, 4.4))
ax = fig.add_subplot()
ax.set_aspect("equal")
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.plot(points[0], points[1], ".", color=style.VEHICLE, markersize=2, alpha=0.4,
        label="poses moved by the motion model")
ax.plot(ellipse[0], ellipse[1], color=style.ESTIMATE, label="EKF 95% ellipse of $\\bar{P}$")
ax.plot(mean[0, 0], mean[1, 0], "x", color=style.ESTIMATE, markersize=9, mew=2,
        label="EKF mean $f(\\hat{\\mathbf{x}})$")
ax.plot(points[0].mean(), points[1].mean(), "+", color=style.ERROR, markersize=11, mew=2,
        label="mean of the poses")
ax.plot(0.0, 0.0, "o", color=style.VEHICLE)
ax.text(0.0, -1.0, "start", color=style.VEHICLE, ha="center")
ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.18), ncol=2, frameon=False, fontsize=10)

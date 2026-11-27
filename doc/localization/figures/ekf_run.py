"""
ekf_run.py

Figure of one run of the extended Kalman filter: the true path, the GNSS
positions, dead reckoning (the noisy inputs integrated alone) and the EKF
estimate with its covariance ellipses. The motion model and the filter are the
ones of the simulation, imported from src/

Author: Shisato Yano
"""

import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

import figure_style as style

COMPONENTS = Path(__file__).absolute().parents[3] / "src" / "components"
sys.path.append(str(COMPONENTS / "state"))
sys.path.append(str(COMPONENTS / "array"))
sys.path.append(str(COMPONENTS / "common"))
sys.path.append(str(COMPONENTS / "localization" / "kalman_filter"))
from state import State
from plot_lib import draw_covariance_ellipse
from extended_kalman_filter_localizer import ExtendedKalmanFilterLocalizer

DT = 0.1           # time step [s], as in the simulation
SPAN = 30.0        # length of the run [s]
GNSS_STD = 1.0     # standard deviation of the GNSS noise [m], as in the simulation
ELLIPSE_EVERY = 50  # steps between two drawn ellipses

np.random.seed(1)
ekf = ExtendedKalmanFilterLocalizer()  # Q and R of the simulation
q_std = np.sqrt(np.diag(ekf.INPUT_NOISE_VAR_MAT)).reshape(2, 1)

true = np.zeros((4, 1))       # [x, y, theta, v], the vehicle starts at rest
dead_reckoning = true.copy()
estimate = np.zeros((4, 1))   # the filter starts at 0 with P = I, as in the code
true_path, dr_path, est_path, gnss_path, ellipses = [], [], [], [], []

for step in range(int(SPAN / DT)):
    t = step * DT
    # Accelerate to about 3 m/s, then weave left and right
    u = np.array([[1.0 if t < 3.0 else 0.0], [0.25 * np.sin(2 * np.pi * t / 12.0)]])

    true = State.motion_model(true, u, DT)
    dead_reckoning = State.motion_model(dead_reckoning, u + q_std * np.random.randn(2, 1), DT)
    gnss = true[:2] + GNSS_STD * np.random.randn(2, 1)
    # The filter adds the input noise itself, so it sees the same kind of inputs as dead reckoning
    last = State(estimate[0, 0], estimate[1, 0], estimate[2, 0], estimate[3, 0])
    estimate = ekf.update(last, u[0, 0], u[1, 0], DT, gnss)

    true_path.append(true[:2, 0])
    dr_path.append(dead_reckoning[:2, 0])
    est_path.append(estimate[:2, 0])
    gnss_path.append(gnss[:, 0])
    if (step + 1) % ELLIPSE_EVERY == 0:
        ellipses.append((estimate[0, 0], estimate[1, 0], ekf.cov_mat[:2, :2].copy()))

true_path, dr_path, est_path, gnss_path = map(np.array, (true_path, dr_path, est_path, gnss_path))
# Root mean square of the position error over the run [m], quoted in the text
RMS_DEAD_RECKONING = float(np.sqrt(np.mean(np.sum((dr_path - true_path) ** 2, axis=1))))
RMS_EKF = float(np.sqrt(np.mean(np.sum((est_path - true_path) ** 2, axis=1))))
RMS_GNSS = float(np.sqrt(np.mean(np.sum((gnss_path - true_path) ** 2, axis=1))))

fig = plt.figure(figsize=(6.4, 4.6))
ax = fig.add_subplot()
ax.set_aspect("equal")
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.plot(gnss_path[:, 0], gnss_path[:, 1], ".", color=style.MEASUREMENT, markersize=3, alpha=0.5,
        label="GNSS $\\mathbf{z}_t$")
ax.plot(true_path[:, 0], true_path[:, 1], color=style.VEHICLE, label="true path")
ax.plot(dr_path[:, 0], dr_path[:, 1], "--", color=style.ERROR, label="dead reckoning")
ax.plot(est_path[:, 0], est_path[:, 1], color=style.ESTIMATE, label="EKF $\\hat{\\mathbf{x}}_t$")
for x, y, cov in ellipses:
    draw_covariance_ellipse(ax, [], x, y, cov, color=style.ESTIMATE, linewidth=1.2)
ax.plot([], [], color=style.ESTIMATE, linewidth=1.2, label="covariance $P_t$ (x, y)")

# Zoom around one ellipse: it is about 1 m across, too small to see at the scale of the run
x0, y0, cov0 = ellipses[3]
zoom = ax.inset_axes([0.62, 0.05, 0.36, 0.36])
zoom.set_aspect("equal")
zoom.plot(gnss_path[:, 0], gnss_path[:, 1], ".", color=style.MEASUREMENT, markersize=4, alpha=0.6)
zoom.plot(true_path[:, 0], true_path[:, 1], color=style.VEHICLE)
zoom.plot(est_path[:, 0], est_path[:, 1], color=style.ESTIMATE)
draw_covariance_ellipse(zoom, [], x0, y0, cov0, color=style.ESTIMATE, linewidth=1.2)
zoom.set_xlim(x0 - 2.5, x0 + 2.5)
zoom.set_ylim(y0 - 2.5, y0 + 2.5)
zoom.set_xticks([])
zoom.set_yticks([])
zoom.spines[["top", "right"]].set_visible(True)
ax.indicate_inset_zoom(zoom, edgecolor=style.COURSE)
ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.18), ncol=3, frameon=False, fontsize=10)

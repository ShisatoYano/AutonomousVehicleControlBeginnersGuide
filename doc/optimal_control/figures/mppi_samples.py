"""
mppi_samples.py

Figure of one MPPI update with equations (1) to (4): steering sequences sampled
around a straight-ahead guess, the trajectories they give, drawn darker the more
weight they get, and the trajectory of the weighted update

Author: Shisato Yano
"""

import numpy as np
import matplotlib.pyplot as plt

import figure_style as style

rng = np.random.default_rng(1)

L, V, DT, N = 2.5, 5.0, 0.1, 20   # wheelbase [m], speed [m/s], time step [s], horizon
M = 60                            # number of samples
SIGMA_STEER = 0.15                # [rad]
MAX_STEER = 0.523                 # [rad]
LAMBDA = 5.0
Y_REF = 2.0                       # the course is the line y = Y_REF, driven in the +x direction
W_Y, W_THETA = 1.0, 1.0           # weights of the lateral error and the heading error


def rollout(steers: np.ndarray) -> np.ndarray:
    """Positions (N+1, 2) with the model of equation (1) at a constant speed"""
    x, y, theta = 0.0, 0.0, 0.0
    points = [(x, y)]
    for steer in steers:
        x += V * np.cos(theta) * DT
        y += V * np.sin(theta) * DT
        theta += V / L * np.tan(steer) * DT
        points.append((x, y))
    return np.array(points)


def cost(points: np.ndarray) -> float:
    """Stage and terminal costs of equation (2): lateral and heading errors to the course"""
    heading = np.arctan2(np.diff(points[:, 1]), np.diff(points[:, 0]))
    return float(W_Y * np.sum((points[1:, 1] - Y_REF) ** 2) + W_THETA * np.sum(heading ** 2))


u = np.zeros(N)                                    # current guess: straight ahead
epsilon = rng.normal(0.0, SIGMA_STEER, (M, N))     # equation (1)
samples = np.clip(u + epsilon, -MAX_STEER, MAX_STEER)
trajectories = [rollout(v) for v in samples]
costs = np.array([cost(t) for t in trajectories])
weights = np.exp(-(costs - costs.min()) / LAMBDA)  # equation (3)
weights /= weights.sum()
u_new = np.clip(u + weights @ epsilon, -MAX_STEER, MAX_STEER)  # equation (4)

fig = plt.figure()
ax = fig.add_subplot()
ax.set_aspect("equal")
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")

ax.axhline(Y_REF, color=style.COURSE, linestyle="--", label="course")
for trajectory, w in zip(trajectories, weights):
    ax.plot(trajectory[:, 0], trajectory[:, 1], color=style.CONTROL,
            linewidth=1.0, alpha=0.1 + 0.9 * w / weights.max())
ax.plot([], [], color=style.CONTROL, linewidth=1.0, label="samples (darker: more weight)")
new = rollout(u_new)
ax.plot(new[:, 0], new[:, 1], color=style.ESTIMATE, linewidth=2.5, label="weighted update")
ax.plot(0.0, 0.0, "o", color=style.VEHICLE)
ax.text(0.0, -0.9, "vehicle", color=style.VEHICLE, ha="center")
ax.set_ylim(-2.5, 2.8)
ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False, fontsize=10)

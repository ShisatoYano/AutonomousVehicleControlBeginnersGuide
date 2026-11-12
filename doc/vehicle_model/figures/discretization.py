"""
discretization.py

Figure of the discretization error of equation (4): the exact circle of a
vehicle driving at a constant speed and yaw rate, and the positions computed
step by step with a small and a large time step

Author: Shisato Yano
"""

import numpy as np
import matplotlib.pyplot as plt

import figure_style as style

V = 5.0       # speed [m/s]
OMEGA = 0.5   # yaw rate [rad/s]
T = 2 * np.pi / OMEGA  # one full turn [s]


def simulate(dt: float) -> np.ndarray:
    """Positions with equation (4) for a = 0: move along the heading of the start of each step"""
    x, y, theta = 0.0, 0.0, 0.0
    points = [(x, y)]
    for _ in range(int(round(T / dt))):
        x += V * np.cos(theta) * dt
        y += V * np.sin(theta) * dt
        theta += OMEGA * dt
        points.append((x, y))
    return np.array(points)


fig = plt.figure()
ax = fig.add_subplot()
ax.set_aspect("equal")

radius = V / OMEGA
angles = np.linspace(0.0, 2 * np.pi, 400)
ax.plot(radius * np.sin(angles), radius * (1 - np.cos(angles)), color=style.COURSE, label="exact circle")

fine = simulate(0.1)
coarse = simulate(1.0)
ax.plot(fine[:, 0], fine[:, 1], ".", color=style.VEHICLE, markersize=3, label=r"$\Delta t = 0.1$ s")
ax.plot(coarse[:, 0], coarse[:, 1], "o-", color=style.ERROR, markersize=4, linewidth=1.0, label=r"$\Delta t = 1.0$ s")

ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.legend(loc="center")

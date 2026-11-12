"""
gnss_noise.py

Figure of the GNSS observation model of equation (5): the true positions of a
vehicle on a curve, the noisy observed positions, and the circles of one and two
standard deviations around one true position

Author: Shisato Yano
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

import figure_style as style

SIGMA_M = 1.0   # standard deviation of the noise in x and y, as in the localization simulations
rng = np.random.default_rng(0)

s = np.linspace(0.0, 30.0, 31)   # one observation per meter of driving
true_xy = np.column_stack([s, 4.0 * np.sin(s / 10.0)])
observed_xy = true_xy + rng.normal(0.0, SIGMA_M, true_xy.shape)

fig = plt.figure()
ax = fig.add_subplot()
ax.set_aspect("equal")
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")

ax.plot(true_xy[:, 0], true_xy[:, 1], color=style.VEHICLE, label="true position")
ax.plot(observed_xy[:, 0], observed_xy[:, 1], ".", color=style.MEASUREMENT, label=r"observation $\mathbf{z}_t$")
for n, alpha in [(1, 0.25), (2, 0.12)]:
    ax.add_patch(Circle(true_xy[15], n * SIGMA_M, color=style.MEASUREMENT, alpha=alpha))
ax.text(true_xy[15, 0] + 0.3, true_xy[15, 1] - 3.0, r"$\sigma$ and $2\sigma$", color=style.MEASUREMENT)
ax.set_ylim(-7.0, 8.0)
ax.legend(loc="lower center", ncol=2, frameon=False)

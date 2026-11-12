"""
frames.py

Figure of the global frame and the vehicle frame: the vehicle's pose
(x, y, theta) in the global frame, and the vehicle frame's x axis forward
and y axis to the left

Author: Shisato Yano
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Arc, Polygon

import figure_style as style

X, Y, THETA = 4.0, 2.5, 0.6   # vehicle pose [m], [m], [rad]
AXIS = 1.8                    # length of the drawn vehicle axes [m]


def arrow(ax, start: np.ndarray, end: np.ndarray, color: str) -> None:
    ax.annotate("", end, start, arrowprops=dict(arrowstyle="->", color=color, lw=1.8))


fig = plt.figure()
ax = fig.add_subplot()
ax.set_aspect("equal")
ax.axis("off")

# Global frame {G}
origin = np.zeros(2)
arrow(ax, origin, np.array([7.5, 0.0]), "black")
arrow(ax, origin, np.array([0.0, 5.0]), "black")
ax.text(7.6, -0.15, r"$x$", fontsize=13)
ax.text(-0.15, 5.1, r"$y$", fontsize=13)
ax.text(-0.6, -0.5, r"global frame $\{G\}$")

# Vehicle body and its pose
forward = np.array([np.cos(THETA), np.sin(THETA)])
left = np.array([-np.sin(THETA), np.cos(THETA)])
pos = np.array([X, Y])
corners = [pos - 0.5 * forward - 0.7 * left, pos + 2.2 * forward - 0.7 * left,
           pos + 2.2 * forward + 0.7 * left, pos - 0.5 * forward + 0.7 * left]
ax.add_patch(Polygon(corners, closed=True, fill=False, edgecolor=style.VEHICLE, alpha=0.5))
ax.plot(*pos, "o", color=style.VEHICLE)
ax.plot([0, X], [Y, Y], ":", color=style.COURSE)
ax.plot([X, X], [0, Y], ":", color=style.COURSE)
ax.text(X - 0.1, -0.45, r"$x$", color=style.VEHICLE)
ax.text(-0.45, Y - 0.1, r"$y$", color=style.VEHICLE)

# Vehicle frame {V}: x forward, y to the left, origin at the rear axle center
arrow(ax, pos, pos + AXIS * forward, style.VEHICLE)
arrow(ax, pos, pos + AXIS * left, style.VEHICLE)
ax.text(*(pos + (AXIS + 0.2) * forward), r"$x_V$ (forward)", color=style.VEHICLE)
ax.text(*(pos + (AXIS + 0.2) * left - [0.3, 0.0]), r"$y_V$ (left)", color=style.VEHICLE)
ax.text(*(pos - 0.9 * left - [0.2, 0.0]), r"vehicle frame $\{V\}$", color=style.VEHICLE)

# Yaw angle, counterclockwise from the global x axis
ax.plot([X, X + 1.6], [Y, Y], "--", color=style.COURSE)
ax.add_patch(Arc(pos, 2.4, 2.4, theta1=0, theta2=np.degrees(THETA), color=style.VEHICLE))
ax.text(X + 1.3, Y + 0.25, r"$\theta$", color=style.VEHICLE)

ax.set_xlim(-1.0, 8.5)
ax.set_ylim(-1.0, 6.0)

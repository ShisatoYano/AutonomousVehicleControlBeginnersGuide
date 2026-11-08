"""
bicycle_model.py

Figure of the kinematic bicycle model: the rear and front wheels, the wheelbase
L, the steering angle delta, the instantaneous center of rotation and the
turning radius R = L / tan(delta)

Author: Shisato Yano
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Arc, Polygon

import figure_style as style

L = 2.5          # wheelbase [m]
DELTA = 0.5      # steering angle [rad]
THETA = 0.15     # yaw angle [rad]
REAR = np.array([0.0, 0.0])

forward = np.array([np.cos(THETA), np.sin(THETA)])
left = np.array([-np.sin(THETA), np.cos(THETA)])
front = REAR + L * forward
radius = L / np.tan(DELTA)
icr = REAR + radius * left


def wheel(center: np.ndarray, angle: float) -> np.ndarray:
    d = np.array([np.cos(angle), np.sin(angle)])
    n = np.array([-np.sin(angle), np.cos(angle)])
    return np.array([center - 0.5 * d - 0.13 * n, center + 0.5 * d - 0.13 * n,
                     center + 0.5 * d + 0.13 * n, center - 0.5 * d + 0.13 * n])


def arrow(ax, start: np.ndarray, end: np.ndarray, color: str) -> None:
    ax.annotate("", end, start, arrowprops=dict(arrowstyle="->", color=color, lw=1.8))


fig = plt.figure()
ax = fig.add_subplot()
ax.set_aspect("equal")
ax.axis("off")

# Body line and wheels
ax.plot(*np.column_stack([REAR, front]), color=style.VEHICLE)
ax.add_patch(Polygon(wheel(REAR, THETA), closed=True, color=style.VEHICLE))
ax.add_patch(Polygon(wheel(front, THETA + DELTA), closed=True, color=style.CONTROL))
ax.text(*(REAR + 0.45 * L * forward + 0.15 * left), r"$L$", color=style.VEHICLE)
ax.text(*(REAR - 0.9 * left - [0.6, 0.1]), r"rear axle $(x, y)$", color=style.VEHICLE)

# Velocities: each wheel moves along its own direction (no slip)
below = REAR - 0.35 * left
arrow(ax, below, below + 1.4 * forward, style.VEHICLE)
ax.text(*(below + 1.5 * forward + [0.0, -0.2]), r"$v$", color=style.VEHICLE)
steer_dir = np.array([np.cos(THETA + DELTA), np.sin(THETA + DELTA)])
arrow(ax, front, front + 1.4 * steer_dir, style.CONTROL)

# Steering angle between the body and the front wheel
ax.plot(*np.column_stack([front, front + 1.3 * forward]), ":", color=style.COURSE)
ax.add_patch(Arc(front, 1.8, 1.8, theta1=np.degrees(THETA), theta2=np.degrees(THETA + DELTA), color=style.CONTROL))
mid = THETA + DELTA / 2
ax.text(*(front + 1.05 * np.array([np.cos(mid), np.sin(mid)])), r"$\delta$", color=style.CONTROL)

# Lines perpendicular to the wheels meet at the instantaneous center of rotation
front_normal = np.array([-np.sin(THETA + DELTA), np.cos(THETA + DELTA)])
ax.plot(*np.column_stack([REAR, icr]), "--", color=style.COURSE)
ax.plot(*np.column_stack([front, icr]), "--", color=style.COURSE)
ax.plot(*icr, "o", color=style.COURSE)
ax.text(*(icr + [0.15, 0.05]), "center of rotation", color=style.COURSE)
ax.text(*(REAR + 0.5 * radius * left + [-0.55, 0.0]), r"$R$", color=style.COURSE)

# Path of the rear axle: a circle of radius R around the center of rotation
start = np.degrees(np.arctan2(*(REAR - icr)[::-1]))
ax.add_patch(Arc(icr, 2 * radius, 2 * radius, theta1=start, theta2=start + 40, color=style.VEHICLE, linestyle="-."))

ax.set_xlim(-2.5, 5.0)
ax.set_ylim(-1.2, icr[1] + 0.8)

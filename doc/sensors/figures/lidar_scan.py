"""
lidar_scan.py

Figure of a simulated LiDAR scan in the sensor frame: the obstacles' outlines,
one beam per angle step, and the nearest point in each step, which hides the
parts of the obstacles behind it

Author: Shisato Yano
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Arc

import figure_style as style

RESO_RAD = np.deg2rad(4.0)   # angle step, coarser than the simulations' 2 deg so the beams can be seen
MAX_RANGE_M = 14.0

# Rectangles as (center x, center y, half length, half width) in the sensor frame [m]
OBSTACLES = [(6.0, 1.0, 0.8, 1.2), (11.0, 5.0, 1.5, 2.5), (-4.0, -6.0, 2.0, 1.0)]


def outline(cx: float, cy: float, hl: float, hw: float) -> np.ndarray:
    """Closed outline of a rectangle, sampled finely like the contour points of the LiDAR model"""
    corners = np.array([[hl, hw], [-hl, hw], [-hl, -hw], [hl, -hw], [hl, hw]]) + [cx, cy]
    t = np.linspace(0.0, 1.0, 200, endpoint=False)[:, None]
    return np.vstack([(1 - t) * a + t * b for a, b in zip(corners[:-1], corners[1:])])


fig = plt.figure()
ax = fig.add_subplot()
ax.set_aspect("equal")
ax.axis("off")

# Keep the nearest outline point in each angle step: equations (1) and (2)
nearest = {}
for cx, cy, hl, hw in OBSTACLES:
    points = outline(cx, cy, hl, hw)
    closed = np.vstack([points, points[:1]])
    ax.plot(closed[:, 0], closed[:, 1], color=style.OBSTACLE, linewidth=1.0)
    for px, py in points:
        r, phi = np.hypot(px, py), np.arctan2(py, px)
        step = int(round((phi % (2 * np.pi)) / RESO_RAD))
        if r <= MAX_RANGE_M and r < nearest.get(step, (np.inf,))[0]:
            nearest[step] = (r, step * RESO_RAD)

scan = np.array([[r * np.cos(phi), r * np.sin(phi)] for r, phi in nearest.values()])
for x, y in scan:
    ax.plot([0.0, x], [0.0, y], color=style.MEASUREMENT, linewidth=0.6, alpha=0.4)
ax.plot(scan[:, 0], scan[:, 1], "o", color=style.MEASUREMENT, markersize=4)

# Sensor frame
ax.annotate("", (2.5, 0.0), (0.0, 0.0), arrowprops=dict(arrowstyle="->", color=style.VEHICLE))
ax.annotate("", (0.0, 2.5), (0.0, 0.0), arrowprops=dict(arrowstyle="->", color=style.VEHICLE))
ax.text(2.6, -0.4, r"$x_S$", color=style.VEHICLE)
ax.text(-0.8, 2.5, r"$y_S$", color=style.VEHICLE)

# Range and bearing of one point
r, phi = min(nearest.values(), key=lambda m: abs(m[1] - np.deg2rad(32.0)))
ax.add_patch(Arc((0.0, 0.0), 6.0, 6.0, theta1=0.0, theta2=np.degrees(phi), color=style.MEASUREMENT))
ax.text(3.1, 0.8, r"$\phi$", color=style.MEASUREMENT)
ax.text(0.55 * r * np.cos(phi) - 0.3, 0.55 * r * np.sin(phi) + 0.3, r"$r$", color=style.MEASUREMENT)

# Maximum range
arc = np.linspace(-0.2, 1.0, 100)
ax.plot(MAX_RANGE_M * np.cos(arc), MAX_RANGE_M * np.sin(arc), color=style.COURSE, linestyle="--", linewidth=1.0)
ax.text(MAX_RANGE_M * np.cos(-0.2) + 0.2, MAX_RANGE_M * np.sin(-0.2), r"$r_{\max}$", color=style.COURSE)
ax.annotate("hidden behind\nthe nearer obstacle", (12.5, 3.0), (13.5, 0.5), color=style.OBSTACLE, fontsize=10,
            arrowprops=dict(arrowstyle="->", color=style.OBSTACLE, linewidth=0.8))

"""
rear_wheel_feedback_errors.py

Figure of the errors of the rear wheel feedback controller: the rear axle, the
nearest point on the course, the lateral error e and the heading error theta_e
of equation (1)

Author: Dhruv Sharma
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Arc, Polygon

import figure_style as style

# Course y = A sin(x / B), vehicle at the rear axle, all lengths in [m]
A, B = 1.5, 5.0
L = 2.5                       # wheelbase
REAR = np.array([2.0, 2.6])   # rear axle position
THETA = 0.55                  # vehicle yaw [rad]


def course_yaw(x: float) -> float:
    return float(np.arctan(A / B * np.cos(x / B)))


def rotate(points: np.ndarray, angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return points @ np.array([[c, s], [-s, c]])


heading = np.array([np.cos(THETA), np.sin(THETA)])
left = np.array([-np.sin(THETA), np.cos(THETA)])
front = REAR + L * heading

# Nearest course point to the rear axle, and the errors of equation (1)
xs = np.linspace(-2.0, 10.0, 2401)
course = np.column_stack([xs, A * np.sin(xs / B)])
nearest = course[np.argmin(np.linalg.norm(course - REAR, axis=1))]
theta_r = course_yaw(nearest[0])
tangent = np.array([np.cos(theta_r), np.sin(theta_r)])

fig = plt.figure()
ax = fig.add_subplot()
ax.set_aspect("equal")
ax.axis("off")

# Course and its tangent at the nearest point
ax.plot(course[:, 0], course[:, 1], color=style.COURSE)
ax.annotate("", nearest + 3.0 * tangent, nearest,
            arrowprops=dict(arrowstyle="->", color=style.COURSE, linestyle="--"))
ax.plot(*nearest, "o", color=style.COURSE)
ax.text(*(nearest + [-2.6, -0.9]), r"$(x_r, y_r)$, curvature $\kappa$", color=style.COURSE)
ax.text(*(nearest + 1.2 * tangent + [0.3, -1.2]), r"course direction $\theta_r$", color=style.COURSE)

# Vehicle body, axles and heading
body = rotate(np.array([[-0.6, -0.8], [L + 0.6, -0.8], [L + 0.6, 0.8], [-0.6, 0.8]]), THETA) + REAR
ax.add_patch(Polygon(body, closed=True, fill=False, edgecolor=style.VEHICLE, alpha=0.5))
ax.plot(*REAR, "o", color=style.VEHICLE)
ax.plot(*front, "o", color=style.VEHICLE)
ax.text(*(REAR + [-1.3, 0.1]), r"$(x, y)$", color=style.VEHICLE)
ax.annotate("", REAR + 3.0 * heading, REAR, arrowprops=dict(arrowstyle="->", color=style.VEHICLE))
ax.text(*(front + 1.6 * heading + [-0.6, 0.3]), r"heading $\theta$", color=style.VEHICLE)

# Heading error between the course direction and the heading, drawn at the rear axle
ax.plot(*np.column_stack([REAR, REAR + 3.0 * tangent]), color=style.COURSE, linestyle=":")
ax.add_patch(Arc(REAR, 4.4, 4.4, theta1=np.degrees(theta_r), theta2=np.degrees(THETA), color=style.ERROR))
mid = (theta_r + THETA) / 2
ax.text(*(REAR + 2.4 * np.array([np.cos(mid), np.sin(mid)])), r"$\theta_e$", color=style.ERROR)

# Lateral error from the course point to the rear axle
ax.annotate("", REAR, nearest, arrowprops=dict(arrowstyle="<->", color=style.ERROR))
ax.text(*((REAR + nearest) / 2 + [0.15, -0.1]), r"$e$", color=style.ERROR)

ax.set_xlim(-1.0, 9.5)
ax.set_ylim(-1.0, 6.0)

"""
stanley_errors.py

Figure of the Stanley controller's errors: the front axle, the nearest point on
the course, the lateral error e, the heading error theta_e and the steering
angle delta of equation (3)

Author: Shisato Yano
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Arc, Polygon

import figure_style as style

# Course y = A sin(x / B), vehicle at the rear axle, all lengths in [m]
A, B = 1.5, 5.0
L = 2.5                       # wheelbase
REAR = np.array([0.5, 1.5])   # rear axle position
THETA = 0.45                  # vehicle yaw [rad]
K, V = 0.5, 3.0               # control gain and speed of equation (3)


def course_y(x: np.ndarray) -> np.ndarray:
    return A * np.sin(x / B)


def course_yaw(x: float) -> float:
    return float(np.arctan(A / B * np.cos(x / B)))


def rotate(points: np.ndarray, angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return points @ np.array([[c, s], [-s, c]])


heading = np.array([np.cos(THETA), np.sin(THETA)])
left = np.array([-np.sin(THETA), np.cos(THETA)])
front = REAR + L * heading

# Nearest course point to the front axle, and the errors of equation (2)
xs = np.linspace(-2.0, 10.0, 2401)
course = np.column_stack([xs, course_y(xs)])
nearest = course[np.argmin(np.linalg.norm(course - front, axis=1))]
theta_r = course_yaw(nearest[0])
theta_e = THETA - theta_r
e = float(left @ (front - nearest))
delta = -(theta_e + np.arctan(K * e / V))

fig = plt.figure()
ax = fig.add_subplot()
ax.set_aspect("equal")
ax.axis("off")

# Course and its tangent at the target point
ax.plot(course[:, 0], course[:, 1], color=style.COURSE)
tangent = np.array([np.cos(theta_r), np.sin(theta_r)])
ax.annotate("", nearest + 2.6 * tangent, nearest,
            arrowprops=dict(arrowstyle="->", color=style.COURSE, linestyle="--"))
ax.plot(*nearest, "o", color=style.COURSE)
ax.text(*(nearest + 1.2 * tangent + [0.0, -0.75]), r"course direction $\theta_r$", color=style.COURSE)

# Vehicle body, axles and heading
body = rotate(np.array([[-0.6, -0.8], [L + 0.6, -0.8], [L + 0.6, 0.8], [-0.6, 0.8]]), THETA) + REAR
ax.add_patch(Polygon(body, closed=True, fill=False, edgecolor=style.VEHICLE, alpha=0.5))
ax.plot(*REAR, "o", color=style.VEHICLE)
ax.plot(*front, "o", color=style.VEHICLE)
ax.text(*(REAR - 0.55 * left - [0.4, 0.15]), r"$(x, y)$", color=style.VEHICLE)
ax.text(*(REAR + 0.5 * L * heading - 0.55 * left), r"$L$", color=style.VEHICLE)
ax.annotate("", front + 2.6 * heading, front,
            arrowprops=dict(arrowstyle="->", color=style.VEHICLE))
ax.text(*(front + 2.7 * heading + [0.05, 0.05]), r"heading $\theta$", color=style.VEHICLE)

# Heading error between the course direction and the heading, drawn at the front axle
ax.add_patch(Arc(front, 4.6, 4.6, theta1=np.degrees(theta_r), theta2=np.degrees(THETA), color=style.ERROR))
mid = (theta_r + THETA) / 2
ax.text(*(front + 2.45 * np.array([np.cos(mid), np.sin(mid)])), r"$\theta_e$", color=style.ERROR)
ax.plot(*np.column_stack([front, front + 2.6 * tangent]), color=style.COURSE, linestyle=":")

# Lateral error from the course point to the front axle
ax.annotate("", front, nearest, arrowprops=dict(arrowstyle="<->", color=style.ERROR))
ax.text(*((front + nearest) / 2 + [0.15, -0.1]), r"$e$", color=style.ERROR)

# Front wheel turned by the steering angle
wheel = rotate(np.array([[-0.45, -0.12], [0.45, -0.12], [0.45, 0.12], [-0.45, 0.12]]), THETA + delta) + front
ax.add_patch(Polygon(wheel, closed=True, color=style.CONTROL))
steer = np.array([np.cos(THETA + delta), np.sin(THETA + delta)])
ax.annotate("", front + 1.8 * steer, front, arrowprops=dict(arrowstyle="->", color=style.CONTROL))
ax.add_patch(Arc(front, 2.0, 2.0, theta1=np.degrees(THETA + delta), theta2=np.degrees(THETA), color=style.CONTROL))
mid_d = THETA + delta / 2
ax.text(*(front + 1.3 * np.array([np.cos(mid_d - 0.6), np.sin(mid_d - 0.6)])), r"$\delta$", color=style.CONTROL)

ax.set_xlim(-1.0, 9.5)
ax.set_ylim(-1.5, 5.0)

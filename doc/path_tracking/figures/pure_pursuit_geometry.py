"""
pure_pursuit_geometry.py

Figure of the Pure Pursuit geometry: the look-ahead point at the distance l_d
from the rear axle, the angle alpha to it, the arc of radius R through it, and
the steering angle delta of equation (4)

Author: Dhruv Sharma
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Arc, Polygon

import figure_style as style

# Course y = A sin(x / B), vehicle at the rear axle, all lengths in [m]
A, B = 1.5, 5.0
L = 2.5                       # wheelbase
REAR = np.array([-4.0, -1.6])  # rear axle position
THETA = -0.15                 # vehicle yaw [rad]
LD = 7.0                      # look-ahead distance


def rotate(points: np.ndarray, angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return points @ np.array([[c, s], [-s, c]])


heading = np.array([np.cos(THETA), np.sin(THETA)])
left = np.array([-np.sin(THETA), np.cos(THETA)])
front = REAR + L * heading

# Look-ahead point: the first course point ahead at the distance l_d from the rear axle
xs = np.linspace(-6.0, 10.0, 3201)
course = np.column_stack([xs, A * np.sin(xs / B)])
ahead = course[(course - REAR) @ heading > 0.0]
target = ahead[np.argmax(np.linalg.norm(ahead - REAR, axis=1) >= LD)]
to_target = target - REAR
alpha = np.arctan2(to_target[1], to_target[0]) - THETA   # equation (1)
R = LD / (2.0 * np.sin(alpha))                           # equation (2)
delta = np.arctan(2.0 * L * np.sin(alpha) / LD)          # equation (4)
center = REAR + R * left

fig = plt.figure()
ax = fig.add_subplot()
ax.set_aspect("equal")
ax.axis("off")

# Course and the look-ahead point
ax.plot(course[:, 0], course[:, 1], color=style.COURSE)
ax.plot(*target, "o", color=style.COURSE, markersize=8)
ax.text(*(target + [0.3, -0.6]), r"look-ahead point $(x_p, y_p)$", color=style.COURSE)
ax.text(6.5, A * np.sin(6.5 / B) + 0.3, "course", color=style.COURSE)

# Arc from the rear axle through the look-ahead point, around the center of rotation
start = np.degrees(np.arctan2(*(REAR - center)[::-1]))
end = np.degrees(np.arctan2(*(target - center)[::-1]))
ax.add_patch(Arc(center, 2 * abs(R), 2 * abs(R), theta1=min(start, end), theta2=max(start, end),
                 color=style.CONTROL, linestyle="--"))
ax.annotate("", REAR, center, arrowprops=dict(arrowstyle="-", color=style.CONTROL, linestyle=":"))
ax.annotate("", target, center, arrowprops=dict(arrowstyle="-", color=style.CONTROL, linestyle=":"))
ax.text(*((REAR + center) / 2 + [0.25, 0.0]), r"$R$", color=style.CONTROL)
ax.text(*((target + center) / 2 + [0.25, 0.0]), r"$R$", color=style.CONTROL)
ax.text(*(center + [0.3, 0.2]), "center of rotation", color=style.CONTROL, fontsize=10)
ax.plot(*center, "o", color=style.CONTROL, markersize=4)

# Chord from the rear axle to the look-ahead point, of length l_d
ax.plot(*np.column_stack([REAR, target]), color=style.ERROR)
ax.text(*(REAR + 0.4 * to_target + [0.3, -0.55]), r"$l_d$", color=style.ERROR)

# Vehicle body, axles and heading
body = rotate(np.array([[-0.6, -0.8], [L + 0.6, -0.8], [L + 0.6, 0.8], [-0.6, 0.8]]), THETA) + REAR
ax.add_patch(Polygon(body, closed=True, fill=False, edgecolor=style.VEHICLE, alpha=0.5))
ax.plot(*REAR, "o", color=style.VEHICLE)
ax.plot(*front, "o", color=style.VEHICLE)
ax.text(*(REAR + [-0.6, -0.75]), r"$(x, y)$", color=style.VEHICLE)
ax.annotate("", REAR + 8.0 * heading, REAR, arrowprops=dict(arrowstyle="->", color=style.VEHICLE))
ax.text(*(REAR + 8.1 * heading + [0.1, -0.2]), r"heading $\theta$", color=style.VEHICLE)

# Angle alpha between the heading and the line to the look-ahead point
ax.add_patch(Arc(REAR, 10.0, 10.0, theta1=np.degrees(THETA), theta2=np.degrees(THETA + alpha), color=style.ERROR))
mid = THETA + alpha / 2
ax.text(*(REAR + 5.3 * np.array([np.cos(mid), np.sin(mid)])), r"$\alpha$", color=style.ERROR)

# Front wheel turned by the steering angle
wheel = rotate(np.array([[-0.45, -0.12], [0.45, -0.12], [0.45, 0.12], [-0.45, 0.12]]), THETA + delta) + front
ax.add_patch(Polygon(wheel, closed=True, color=style.CONTROL))
ax.text(*(front + [-0.1, 0.4]), r"$\delta$", color=style.CONTROL)

ax.set_xlim(-6.0, 9.0)
ax.set_ylim(-3.0, center[1] + 0.8)

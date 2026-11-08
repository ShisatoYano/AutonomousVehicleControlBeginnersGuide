"""
cubic_spline_course.py

Figure of a 2D cubic spline course: the course through the given points with
its yaw angle theta_r, and its curvature kappa along the distance s,
computed with equations (1) to (8) of the cubic spline section

Author: Dhruv Sharma
"""

import numpy as np
import matplotlib.pyplot as plt

import figure_style as style

# Same points as the cubic_spline and path tracking simulations [m]
X_POINTS = [0.0, 10.0, 25.0, 40.0, 50.0]
Y_POINTS = [0.0, 4.0, -12.0, 20.0, -13.0]


def natural_spline(s: np.ndarray, q: np.ndarray) -> tuple[np.ndarray, ...]:
    """Coefficients a, b, c, d of the natural cubic spline through (s_i, q_i), equations (3) to (5)"""
    n = len(s) - 1
    h = np.diff(s)
    A = np.zeros((n + 1, n + 1))
    r = np.zeros(n + 1)
    A[0, 0] = A[n, n] = 1.0  # natural ends: c_0 = c_n = 0
    for i in range(1, n):
        A[i, i - 1], A[i, i], A[i, i + 1] = h[i - 1], 2.0 * (h[i - 1] + h[i]), h[i]
        r[i] = 3.0 * (q[i + 1] - q[i]) / h[i] - 3.0 * (q[i] - q[i - 1]) / h[i - 1]
    c = np.linalg.solve(A, r)
    b = (q[1:] - q[:-1]) / h - h * (2.0 * c[:-1] + c[1:]) / 3.0
    d = (c[1:] - c[:-1]) / (3.0 * h)
    return q[:-1], b, c[:-1], d


def evaluate(s_knots: np.ndarray, coef: tuple[np.ndarray, ...], s: np.ndarray) -> tuple[np.ndarray, ...]:
    """Value, first and second derivative of the spline at s, equation (1)"""
    a, b, c, d = coef
    i = np.clip(np.searchsorted(s_knots, s, side="right") - 1, 0, len(a) - 1)
    ds = s - s_knots[i]
    q = a[i] + b[i] * ds + c[i] * ds**2 + d[i] * ds**3
    dq = b[i] + 2.0 * c[i] * ds + 3.0 * d[i] * ds**2
    ddq = 2.0 * c[i] + 6.0 * d[i] * ds
    return q, dq, ddq


xp, yp = np.array(X_POINTS), np.array(Y_POINTS)
s_knots = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(xp), np.diff(yp)))])  # equation (6)
s = np.linspace(0.0, s_knots[-1], 1001)
x, dx, ddx = evaluate(s_knots, natural_spline(s_knots, xp), s)
y, dy, ddy = evaluate(s_knots, natural_spline(s_knots, yp), s)
yaw = np.arctan2(dy, dx)                                    # equation (7)
curvature = (dx * ddy - dy * ddx) / (dx**2 + dy**2) ** 1.5  # equation (8)

fig, (ax_xy, ax_k) = plt.subplots(2, 1, figsize=(6.4, 6.4), height_ratios=[1.6, 1.0])

# Course, given points and the yaw angle at a few places
ax_xy.plot(x, y, color=style.COURSE, label="course $(x(s), y(s))$")
ax_xy.plot(xp, yp, "x", color=style.VEHICLE, markersize=9, mew=2, label="given points $(x_i, y_i)$")
for j in range(60, len(s), 120):
    ax_xy.annotate("", (x[j] + 4.0 * np.cos(yaw[j]), y[j] + 4.0 * np.sin(yaw[j])), (x[j], y[j]),
                   arrowprops=dict(arrowstyle="->", color=style.COURSE))
ax_xy.text(x[420] + 2.0, y[420] - 1.0, r"yaw $\theta_r$", color=style.COURSE)
ax_xy.set_aspect("equal")
ax_xy.set_xlabel("$x$ [m]")
ax_xy.set_ylabel("$y$ [m]")
ax_xy.legend(loc="lower left", fontsize=10)

# Curvature along the course, positive in left turns
ax_k.axhline(0.0, color="black", linewidth=0.8)
ax_k.plot(s, curvature, color=style.COURSE)
for sk in s_knots:
    ax_k.axvline(sk, color=style.VEHICLE, linestyle=":", linewidth=1.0)
ax_k.set_xlabel("distance along the course $s$ [m]  (dotted: $s_i$ of the given points)")
ax_k.set_ylabel(r"curvature $\kappa$ [1/m]")
fig.tight_layout()

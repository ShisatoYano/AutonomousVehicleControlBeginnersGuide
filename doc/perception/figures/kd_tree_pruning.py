"""
kd_tree_pruning.py

Figure of a nearest neighbor search in the kd-tree of kd_tree_build.py: the
query point, the circle of the best distance d_min, and the cells that the
test of equation (7) skips. The search is run here, so the skipped cells are
the ones the equations give

Author: hskl18
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle

import figure_style as style

# Source points p_1 ... p_7 and the query point q, in [m]
POINTS = np.array([[1.0, 3.0], [2.0, 6.0], [3.0, 1.0], [5.0, 4.0],
                   [6.0, 7.0], [7.0, 2.0], [9.0, 5.0]])
QUERY = np.array([6.5, 3.0])
X_MIN, X_MAX, Y_MIN, Y_MAX = 0.0, 10.0, 0.0, 8.0


def build(indices: list[int], depth: int = 0) -> dict | None:
    """Median split on the axis depth % 2. Returns a node as a dict"""

    if not indices:
        return None
    axis = depth % 2
    indices = sorted(indices, key=lambda i: POINTS[i, axis])
    median = len(indices) // 2
    return {"index": indices[median], "axis": axis, "depth": depth,
            "left": build(indices[:median], depth + 1),
            "right": build(indices[median + 1:], depth + 1)}


def split_box(node: dict, box: tuple[float, float, float, float]) -> tuple[tuple, tuple]:
    """The cells of the left and right child of a node"""

    x_min, x_max, y_min, y_max = box
    x, y = POINTS[node["index"]]
    if node["axis"] == 0:
        return (x_min, x, y_min, y_max), (x, x_max, y_min, y_max)
    return (x_min, x_max, y_min, y), (x_min, x_max, y, y_max)


def search(node: dict | None, box: tuple, best: list, visited: list, skipped: list) -> None:
    """Nearest neighbor search of the Algorithm section. best is [d_min, index]; skipped collects the cells not visited"""

    if node is None:
        return
    visited.append(node["index"])
    point = POINTS[node["index"]]
    distance = float(np.linalg.norm(point - QUERY))
    if distance < best[0]:
        best[:] = [distance, node["index"]]

    axis = node["axis"]
    left_box, right_box = split_box(node, box)
    if QUERY[axis] <= point[axis]:
        near, far = (node["left"], left_box), (node["right"], right_box)
    else:
        near, far = (node["right"], right_box), (node["left"], left_box)

    search(near[0], near[1], best, visited, skipped)
    # Equation (7): the far side can hold a closer point only if the splitting
    # line is closer than the best distance
    if abs(QUERY[axis] - point[axis]) < best[0]:
        search(far[0], far[1], best, visited, skipped)
    elif far[0] is not None:
        skipped.append(far[1])


def draw_lines(ax, node: dict | None, box: tuple) -> None:
    if node is None:
        return
    x_min, x_max, y_min, y_max = box
    x, y = POINTS[node["index"]]
    line_width = 2.2 - 0.6 * node["depth"]
    if node["axis"] == 0:
        ax.plot([x, x], [y_min, y_max], color=style.COURSE, linewidth=line_width)
    else:
        ax.plot([x_min, x_max], [y, y], color=style.COURSE, linewidth=line_width)
    left_box, right_box = split_box(node, box)
    draw_lines(ax, node["left"], left_box)
    draw_lines(ax, node["right"], right_box)


root = build(list(range(len(POINTS))))
best = [np.inf, -1]
visited: list[int] = []
skipped: list[tuple] = []
search(root, (X_MIN, X_MAX, Y_MIN, Y_MAX), best, visited, skipped)
d_min, nearest = best
# The search must agree with comparing the query with every point
assert nearest == int(np.argmin(np.linalg.norm(POINTS - QUERY, axis=1)))

fig = plt.figure()
ax = fig.add_subplot()
ax.set_aspect("equal")

# Skipped cells
for x_min, x_max, y_min, y_max in skipped:
    ax.add_patch(Rectangle((x_min, y_min), x_max - x_min, y_max - y_min, color=style.COURSE, alpha=0.18, linewidth=0, zorder=0))
    ax.text(x_max - 0.3, y_max - 0.6, "skipped", ha="right", color=style.COURSE)
draw_lines(ax, root, (X_MIN, X_MAX, Y_MIN, Y_MAX))

# Source points, the visited ones filled
for i, (x, y) in enumerate(POINTS):
    ax.plot(x, y, "o", color=style.MEASUREMENT,
            markerfacecolor=style.MEASUREMENT if i in visited else "white")
    ax.text(x + 0.2, y + 0.25, rf"$\mathbf{{p}}_{i + 1}$", color=style.MEASUREMENT)

# Query point, the best distance and its circle
ax.add_patch(Circle(QUERY, d_min, fill=False, color=style.ERROR, linestyle="--"))
ax.plot(*np.column_stack([QUERY, POINTS[nearest]]), color=style.ERROR)
ax.plot(*QUERY, "x", color=style.ERROR, markersize=9, markeredgewidth=2.2)
ax.text(QUERY[0] - 0.75, QUERY[1] - 0.2, r"$\mathbf{q}$", color=style.ERROR)
ax.text(QUERY[0] - 0.8, QUERY[1] + 0.3, r"$d_{\min}$", color=style.ERROR)

# Distance from the query to the splitting line of the root, which is longer than d_min
root_x = POINTS[root["index"], 0]
ax.annotate("", (root_x, QUERY[1] - 1.7), (QUERY[0], QUERY[1] - 1.7),
            arrowprops=dict(arrowstyle="<->", color=style.COURSE))
ax.plot([QUERY[0], QUERY[0]], [QUERY[1], QUERY[1] - 1.7], color=style.COURSE, linestyle=":", linewidth=1.0)
ax.text((root_x + QUERY[0]) / 2, QUERY[1] - 2.3, r"$|q_j - s_j|$", ha="center", color=style.COURSE)

ax.set_xlim(X_MIN, X_MAX)
ax.set_ylim(Y_MIN, Y_MAX)
ax.set_xlabel(r"$x$ [m]")
ax.set_ylabel(r"$y$ [m]")

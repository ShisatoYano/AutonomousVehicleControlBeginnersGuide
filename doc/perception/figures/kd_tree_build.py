"""
kd_tree_build.py

Figure of a kd-tree built from seven points: the splitting lines in the plane
on the left, and the same tree as nodes on the right. The tree is built with
the median split on the axis of equation (3)

Author: hskl18
"""

import numpy as np
import matplotlib.pyplot as plt

import figure_style as style

# Source points p_1 ... p_7, in [m]
POINTS = np.array([[1.0, 3.0], [2.0, 6.0], [3.0, 1.0], [5.0, 4.0],
                   [6.0, 7.0], [7.0, 2.0], [9.0, 5.0]])
X_MIN, X_MAX, Y_MIN, Y_MAX = 0.0, 10.0, 0.0, 8.0
AXIS_NAMES = ("x", "y")


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


def draw_cells(ax, node: dict | None, box: tuple[float, float, float, float]) -> None:
    """Draw the splitting line of each node inside the cell it divides"""

    if node is None:
        return
    x_min, x_max, y_min, y_max = box
    x, y = POINTS[node["index"]]
    line_width = 2.2 - 0.6 * node["depth"]
    if node["axis"] == 0:
        ax.plot([x, x], [y_min, y_max], color=style.COURSE, linewidth=line_width)
        draw_cells(ax, node["left"], (x_min, x, y_min, y_max))
        draw_cells(ax, node["right"], (x, x_max, y_min, y_max))
    else:
        ax.plot([x_min, x_max], [y, y], color=style.COURSE, linewidth=line_width)
        draw_cells(ax, node["left"], (x_min, x_max, y_min, y))
        draw_cells(ax, node["right"], (x_min, x_max, y, y_max))


def draw_tree(ax, node: dict | None, x: float, width: float, parent: tuple[float, float] | None) -> None:
    """Draw the nodes of the tree, the left child on the left"""

    if node is None:
        return
    y = -float(node["depth"])
    if parent:
        ax.plot([parent[0], x], [parent[1], y], color=style.COURSE, zorder=1)
    ax.plot(x, y, "o", markersize=26, markerfacecolor="white",
            markeredgecolor=style.MEASUREMENT, zorder=2)
    ax.text(x, y, rf"$\mathbf{{p}}_{node['index'] + 1}$", ha="center", va="center",
            color=style.MEASUREMENT, zorder=3)
    draw_tree(ax, node["left"], x - width, width / 2, (x, y))
    draw_tree(ax, node["right"], x + width, width / 2, (x, y))


root = build(list(range(len(POINTS))))

fig, (ax_plane, ax_tree) = plt.subplots(1, 2, figsize=(9.6, 4.0))

# Left: the plane, cut by the splitting line of every node
draw_cells(ax_plane, root, (X_MIN, X_MAX, Y_MIN, Y_MAX))
ax_plane.plot(POINTS[:, 0], POINTS[:, 1], "o", color=style.MEASUREMENT)
for i, (x, y) in enumerate(POINTS):
    ax_plane.text(x + 0.2, y + 0.25, rf"$\mathbf{{p}}_{i + 1}$", color=style.MEASUREMENT)
ax_plane.set_xlim(X_MIN, X_MAX)
ax_plane.set_ylim(Y_MIN, Y_MAX)
ax_plane.set_aspect("equal")
ax_plane.set_xlabel(r"$x$ [m]")
ax_plane.set_ylabel(r"$y$ [m]")

# Right: the same tree, one row per depth
draw_tree(ax_tree, root, 0.0, 2.0, None)
for depth in range(3):
    ax_tree.text(4.3, -depth, rf"depth {depth}: split on ${AXIS_NAMES[depth % 2]}$",
                 va="center", color=style.COURSE)
ax_tree.set_xlim(-3.8, 8.6)
ax_tree.set_ylim(-2.6, 0.6)
ax_tree.axis("off")

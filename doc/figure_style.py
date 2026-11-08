"""
figure_style.py

Common style of the figures in the book, used by the figure scripts through
the plot directive (see plot_rcparams in conf.py)

Author: Shisato Yano
"""

# Colors with fixed meanings, so the same thing has the same color in every figure
COURSE = "#7a7a7a"     # reference course and target points
VEHICLE = "#1f5fa8"    # vehicle body, axles and heading
ERROR = "#c0392b"      # errors (lateral error, heading error)
CONTROL = "#2e8b57"    # control inputs (steering angle, acceleration)
ESTIMATE = "#e08a1e"   # estimates and their uncertainty

RC_PARAMS = {
    "figure.figsize": (6.4, 4.0),
    "figure.dpi": 100,
    "savefig.bbox": "tight",
    "font.size": 12,
    "axes.titlesize": 12,
    "axes.labelsize": 12,
    "lines.linewidth": 1.8,
    "axes.spines.top": False,
    "axes.spines.right": False,
}

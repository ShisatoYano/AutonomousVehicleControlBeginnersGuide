"""
social_preview.py

Draws the social preview image (1280x640 px): the title of the project and a
frame of each demo shown at the top of the README. The book build saves it in
doc/_extra, and it is published at the root of the web book

Author: Shisato Yano
"""

from pathlib import Path

import matplotlib.pyplot as plt
from PIL import Image, ImageChops

REPO_DIR = Path(__file__).absolute().parent.parent

# The same demos, in the same order, as the grid at the top of README.md
DEMOS = [
    ("Localization: EKF vs UKF vs PF", "localization/ekf_ukf_pf_comparison/ekf_ukf_pf_comparison.gif"),
    ("Path tracking: MPPI", "path_tracking/mppi_path_tracking/mppi_path_tracking.gif"),
    ("Path planning: Hybrid A*", "path_planning/astar_hybrid_path_planning/astar_hybrid_navigate.gif"),
    ("Mapping: NDT", "mapping/ndt_map_construction/ndt_map_construction.gif"),
    ("Mapping: cost map", "mapping/cost_grid_map_construction/cost_grid_map_construction.gif"),
    ("Perception: rectangle fitting", "perception/point_cloud_rectangle_fitting/point_cloud_rectangle_fitting.gif"),
]
TITLE = "Autonomous Vehicle Control Beginners Guide"
SUBTITLE = "Learn self-driving car algorithms by building them: Python simulations and an open textbook"


def frame(gif_path: Path) -> Image.Image:
    """A frame two thirds into the demo, when the result is mostly drawn, without its white margins"""
    gif = Image.open(gif_path)
    gif.seek(gif.n_frames * 2 // 3)
    image = gif.convert("RGB")
    # Some demos have an off-white background, so nearly white counts as margin too
    difference = ImageChops.difference(image, Image.new("RGB", image.size, "white")).convert("L")
    content = difference.point(lambda value: 255 if value > 16 else 0).getbbox()
    return image.crop(content) if content else image


def draw(out_path: Path) -> None:
    """Save the 1280x640 px preview image to out_path"""
    fig = plt.figure(figsize=(12.8, 6.4), dpi=100, facecolor="white")
    fig.text(0.5, 0.94, TITLE, ha="center", va="center", fontsize=30, fontweight="bold", color="#1f2933")
    fig.text(0.5, 0.875, SUBTITLE, ha="center", va="center", fontsize=15, color="#52606d")

    columns = 3
    for i, (label, gif) in enumerate(DEMOS):
        left = 0.02 + (i % columns) * 0.33
        bottom = 0.43 - (i // columns) * 0.41
        ax = fig.add_axes((left, bottom, 0.29, 0.36))
        ax.imshow(frame(REPO_DIR / "src" / "simulations" / gif))
        ax.set_axis_off()
        fig.text(left + 0.145, bottom + 0.375, label, ha="center", fontsize=12, color="#1f2933")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=100, facecolor="white")
    plt.close(fig)


if __name__ == "__main__":
    draw(Path(__file__).absolute().parent / "_extra" / "social_preview.png")

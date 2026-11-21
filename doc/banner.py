"""
banner.py

Draws the banner of the book's front page (1280x360 px): a frame of the first four demos
shown at the top of the README. The book build saves it in doc/_extra

Author: Shisato Yano
"""

from pathlib import Path

import matplotlib.pyplot as plt
from PIL import Image, ImageChops

REPO_DIR = Path(__file__).absolute().parent.parent

# The same demos, in the same order, as the grid at the top of README.md (the banner shows the first four)
DEMOS = [
    ("Localization: EKF vs UKF vs PF", "localization/ekf_ukf_pf_comparison/ekf_ukf_pf_comparison.gif"),
    ("Path tracking: MPPI", "path_tracking/mppi_path_tracking/mppi_path_tracking.gif"),
    ("Path planning: Hybrid A*", "path_planning/astar_hybrid_path_planning/astar_hybrid_navigate.gif"),
    ("Mapping: NDT", "mapping/ndt_map_construction/ndt_map_construction.gif"),
    ("Mapping: cost map", "mapping/cost_grid_map_construction/cost_grid_map_construction.gif"),
    ("Perception: rectangle fitting", "perception/point_cloud_rectangle_fitting/point_cloud_rectangle_fitting.gif"),
]


def frame(gif_path: Path) -> Image.Image:
    """A frame two thirds into the demo, when the result is mostly drawn, without its white margins"""
    gif = Image.open(gif_path)
    gif.seek(gif.n_frames * 2 // 3)
    image = gif.convert("RGB")
    # Some demos have an off-white background, so nearly white counts as margin too
    difference = ImageChops.difference(image, Image.new("RGB", image.size, "white")).convert("L")
    content = difference.point(lambda value: 255 if value > 16 else 0).getbbox()
    return image.crop(content) if content else image


def draw_banner(out_path: Path) -> None:
    """Save the banner of the front page (1280x360 px): the first four demos"""
    fig = plt.figure(figsize=(12.8, 3.6), dpi=100, facecolor="white")
    for i, (label, gif) in enumerate(DEMOS[:4]):
        left = 0.01 + i * 0.25
        ax = fig.add_axes((left, 0.02, 0.23, 0.74))
        ax.imshow(frame(REPO_DIR / "src" / "simulations" / gif))
        ax.set_axis_off()
        # The banner is shown at about half its width, so the labels are large and on two lines
        fig.text(left + 0.115, 0.88, label.replace(": ", "\n"), ha="center", va="center",
                 fontsize=20, linespacing=1.3, color="#1f2933")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=100, facecolor="white")
    plt.close(fig)


if __name__ == "__main__":
    extra_dir = Path(__file__).absolute().parent / "_extra"
    draw_banner(extra_dir / "banner.png")

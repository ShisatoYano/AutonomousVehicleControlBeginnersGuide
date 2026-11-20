"""
social_preview.py

Draws the social preview image (1280x640 px): the title of the project and a
frame of the first four demos shown at the top of the README. The book build saves it in
doc/_extra, and it is published at the root of the web book. Also draws the banner of the
book's front page (the same demos without the title) and the logo of its sidebar

Author: Shisato Yano
"""

from pathlib import Path

import matplotlib.pyplot as plt
from PIL import Image, ImageChops

REPO_DIR = Path(__file__).absolute().parent.parent

# The same demos, in the same order, as the grid at the top of README.md (the image shows the first four)
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
    # GitHub's repository cards show the image at about 2.6:1, cropping the top and bottom
    # 80 px of 640, so everything stays between y = 100 and 540 px
    fig.text(0.5, 0.81, TITLE, ha="center", va="center", fontsize=30, fontweight="bold", color="#1f2933")
    fig.text(0.5, 0.745, SUBTITLE, ha="center", va="center", fontsize=15, color="#52606d")

    for i, (label, gif) in enumerate(DEMOS[:4]):
        left = 0.02 + i * 0.245
        ax = fig.add_axes((left, 0.17, 0.225, 0.5))
        ax.imshow(frame(REPO_DIR / "src" / "simulations" / gif))
        ax.set_axis_off()
        fig.text(left + 0.1125, 0.685, label, ha="center", fontsize=12, color="#1f2933")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=100, facecolor="white")
    plt.close(fig)


def draw_banner(out_path: Path) -> None:
    """Save the banner of the front page (1280x360 px): the four demos of the preview, without the title"""
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


def draw_logo(out_path: Path) -> None:
    """
    Save the logo of the sidebar (600x300 px, transparent): a car on a curved road that
    scans with a LiDAR, detects an obstacle and follows its planned path
    """
    import numpy as np

    fig = plt.figure(figsize=(6.0, 3.0), dpi=100)
    ax = fig.add_axes((0, 0, 1, 1))
    ax.set_xlim(0, 8)
    ax.set_ylim(-1.7, 2.3)
    ax.set_aspect("equal")
    ax.set_axis_off()

    def course_y(x: np.ndarray | float) -> np.ndarray | float:
        return np.sin(0.7 * x - 0.6)

    # The sidebar's header is blue, so the road is a light band and the sensing is drawn in white and yellow
    x = np.linspace(-0.5, 8.5, 200)
    ax.plot(x, course_y(x), color="#dbe9f6", linewidth=56, solid_capstyle="butt")

    # Planned path: dots along the course ahead of the car
    x_path = np.arange(3.5, 8.2, 0.3)
    ax.plot(x_path, course_y(x_path), "o", color="#1f2933", markersize=4.5, zorder=2)

    # Obstacle beside the road ahead, with the box that the detector fits to the LiDAR points
    obstacle = (5.6, 0.35, 0.9, 0.6)  # left, bottom, width, height
    left, bottom, width, height = obstacle
    ax.fill([left, left + width, left + width, left], [bottom, bottom, bottom + height, bottom + height],
            color="#52606d", zorder=2)
    ax.plot([left - 0.12, left + width + 0.12, left + width + 0.12, left - 0.12, left - 0.12],
            [bottom - 0.12, bottom - 0.12, bottom + height + 0.12, bottom + height + 0.12, bottom - 0.12],
            color="#ffcf33", linewidth=2.5, linestyle=(0, (4, 3)), zorder=4)

    x0 = 2.6
    y0 = course_y(x0)
    yaw = np.arctan(0.7 * np.cos(0.7 * x0 - 0.6))

    # LiDAR: rays from the front of the car, stopped where they hit the obstacle
    sensor = np.array([x0 + 0.75 * np.cos(yaw), y0 + 0.75 * np.sin(yaw)])
    for angle in yaw + np.radians(np.linspace(-40, 40, 17)):
        direction = np.array([np.cos(angle), np.sin(angle)])
        end, hit = sensor + 2.6 * direction, False
        for distance in np.arange(0.0, 2.6, 0.02):
            px, py = sensor + distance * direction
            if left <= px <= left + width and bottom <= py <= bottom + height:
                end, hit = np.array([px, py]), True
                break
        ax.plot([sensor[0], end[0]], [sensor[1], end[1]], color="white", alpha=0.55, linewidth=1.2, zorder=3)
        if hit:
            ax.plot(*end, "o", color="#ffcf33", markersize=5, zorder=5)

    # The car: body and tires in the vehicle's frame, turned to the course's heading
    rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])

    def place(outline_xy: list[list[float]]) -> tuple[np.ndarray, np.ndarray]:
        points = rotation @ (0.7 * np.array(outline_xy))
        return points[0] + x0, points[1] + y0

    for tire_x, tire_y in [(0.75, 0.45), (0.75, -0.45), (-0.75, 0.45), (-0.75, -0.45)]:
        tx, ty = place([[tire_x - 0.25, tire_x + 0.25, tire_x + 0.25, tire_x - 0.25],
                        [tire_y - 0.1, tire_y - 0.1, tire_y + 0.1, tire_y + 0.1]])
        ax.fill(tx, ty, color="#1f2933", zorder=6)
    bx, by = place([[-1.1, 1.1, 1.1, -1.1], [-0.42, -0.42, 0.42, 0.42]])
    ax.fill(bx, by, color="#e8505b", edgecolor="#1f2933", linewidth=2.5, zorder=6)
    wx, wy = place([[0.25, 0.65, 0.65, 0.25], [-0.32, -0.32, 0.32, 0.32]])
    ax.fill(wx, wy, color="#dbe9f6", zorder=6)
    # The LiDAR on the roof
    lx, ly = place([[-0.15], [0.0]])
    ax.plot(lx, ly, "o", color="#1f2933", markersize=9, zorder=7)
    ax.plot(lx, ly, "o", color="#ffcf33", markersize=4, zorder=8)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=100, transparent=True)
    plt.close(fig)


if __name__ == "__main__":
    extra_dir = Path(__file__).absolute().parent / "_extra"
    draw(extra_dir / "social_preview.png")
    draw_banner(extra_dir / "banner.png")
    draw_logo(extra_dir / "logo.png")

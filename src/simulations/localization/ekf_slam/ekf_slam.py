"""
ekf_slam.py

EKF-SLAM (Extended Kalman Filter based Simultaneous Localization and Mapping).
Real-time landmark-based SLAM in 2D with range-bearing sensor.
Demonstrates loop closure via covariance convergence when re-observing landmarks.
"""

# import path setting
import sys
import numpy as np
from pathlib import Path
from math import pi

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"

sys.path.append(abs_dir_path + relative_path + "visualization")
sys.path.append(abs_dir_path + relative_path + "state")
sys.path.append(abs_dir_path + relative_path + "vehicle")
sys.path.append(abs_dir_path + relative_path + "sensors")
sys.path.append(abs_dir_path + relative_path + "localization/ekf_slam")
sys.path.append(abs_dir_path + relative_path + "sensors/landmark_range_bearing")
sys.path.append(abs_dir_path + relative_path + "landmark")

import matplotlib.pyplot as plt

# import component modules
from global_xy_visualizer import GlobalXYVisualizer
from min_max import MinMax
from time_parameters import TimeParameters
from vehicle_specification import VehicleSpecification
from state import State
from four_wheels_vehicle import FourWheelsVehicle
from sensors import Sensors
from ekf_slam_vehicle_localizer import EKFSLAMVehicleLocalizer
from landmark_range_bearing_sensor import LandmarkRangeBearingSensor
from landmark import Landmark
from landmark_list import LandmarkList

# flag to show plot figure
# when executed as unit test, this flag is set as false
show_plot = True

SIGMA_R = 0.2
SIGMA_PHI = 0.1
SIGMA_V = 0.15
SIGMA_OMEGA = 0.08
GATE_THRESHOLD = 2.0
MAX_RANGE = 15.0
NUM_LANDMARKS = 18
INTERVAL_SEC = 0.2
# Min distance between landmarks (so they stay distinct)
MIN_LANDMARK_SPACING = 4.5
# Stop after one lap (circumference / speed); slight overshoot for loop closure
LAP_OVERSHOOT = 1.2


class CircularMotionController:
    """
    Minimal controller to keep the vehicle on a circle at constant speed.
    """

    def __init__(self, radius=10.0, speed=1.0):
        self.radius = radius
        self.speed = speed

    def update(self, state, time_s):
        return None

    def draw(self, axes, elems):
        return None

    def get_target_accel_mps2(self):
        return 0.0

    def get_target_yaw_rate_rps(self):
        return -self.speed / self.radius

    def get_target_steer_rad(self):
        return 0.0


class LegendOverlay:
    """
    Draw legend entries for the true and estimated vehicles.
    """

    def update(self, _time_s):
        pass

    def draw(self, axes, elems):
        true_plot, = axes.plot([], [], "b-", linewidth=2, label="True pose")
        est_plot, = axes.plot([], [], "g-", linewidth=2, label="Est. pose")
        elems.extend([true_plot, est_plot])
        axes.legend(loc="upper left")


def draw_final_comparison(landmark_list, true_state, est_state, localizer):
    """
    After simulation ends, show a side-by-side: true map vs estimated map.
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
    fig.suptitle("EKF-SLAM: True map vs estimated map", fontsize=14)

    # Left: true map
    ax1.set_title("True map (ground truth)")
    ax1.set_aspect("equal")
    xy = landmark_list.as_xy_tuples()
    ax1.plot(
        [p[0] for p in xy],
        [p[1] for p in xy],
        "ko",
        markersize=8,
        label="Landmarks",
    )
    ax1.plot(
        true_state.x_history,
        true_state.y_history,
        "b-",
        linewidth=1.5,
        alpha=0.8,
        label="True path",
    )
    ax1.set_xlabel("X [m]")
    ax1.set_ylabel("Y [m]")
    ax1.legend(loc="upper right")
    ax1.grid(True, alpha=0.3)

    # Right: estimated map
    ax2.set_title("Estimated map (EKF-SLAM)")
    ax2.set_aspect("equal")
    est_lm = localizer.get_estimated_landmarks()
    if est_lm:
        ax2.plot(
            [p[0] for p in est_lm],
            [p[1] for p in est_lm],
            "rx",
            markersize=8,
            label="Est. landmarks",
        )
    ax2.plot(
        est_state.x_history,
        est_state.y_history,
        "g-",
        linewidth=1.5,
        alpha=0.8,
        label="Est. path",
    )
    ax2.set_xlabel("X [m]")
    ax2.set_ylabel("Y [m]")
    ax2.legend(loc="upper right")
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


def main():
    """
    Main process function.
    """
    np.random.seed(42)

    # set simulation parameters
    x_lim, y_lim = MinMax(-5, 55), MinMax(-20, 25)
    spec = VehicleSpecification(area_size=20.0, x_lim=x_lim, y_lim=y_lim)

    center_x, center_y = 25.0, 2.5
    radius_lm = 20.0
    speed = 1.0
    lap_time_sec = (2 * pi * radius_lm / speed) * LAP_OVERSHOOT
    time_params = TimeParameters(span_sec=lap_time_sec, interval_sec=INTERVAL_SEC)
    vis = GlobalXYVisualizer(x_lim, y_lim, time_params, show_zoom=True)

    # create landmark map
    margin = 5.0
    x_min, x_max = x_lim.min_value() + margin, x_lim.max_value() - margin
    y_min, y_max = y_lim.min_value() + margin, y_lim.max_value() - margin
    landmark_coords = []
    max_attempts = 500
    for _ in range(NUM_LANDMARKS):
        for _ in range(max_attempts):
            x, y = float(np.random.uniform(x_min, x_max)), float(np.random.uniform(y_min, y_max))
            if all(
                (x - lx) ** 2 + (y - ly) ** 2 >= MIN_LANDMARK_SPACING ** 2
                for (lx, ly) in landmark_coords
            ):
                landmark_coords.append((x, y))
                break
        else:
            landmark_coords.append((float(np.random.uniform(x_min, x_max)), float(np.random.uniform(y_min, y_max))))

    landmark_list = LandmarkList()
    for x, y in landmark_coords:
        landmark_list.add_landmark(Landmark(x, y))
    vis.add_object(landmark_list)

    # create vehicle's state instances
    est_state = State(
        x_m=center_x + radius_lm,
        y_m=center_y,
        yaw_rad=-pi / 2,
        speed_mps=speed,
        color="g",
    )
    true_state = State(
        x_m=center_x + radius_lm,
        y_m=center_y,
        yaw_rad=-pi / 2,
        speed_mps=speed,
        color="b",
    )

    # create controller, sensor, localizer and vehicles
    controller = CircularMotionController(radius=radius_lm, speed=speed)
    landmark_sensor = LandmarkRangeBearingSensor(SIGMA_R, SIGMA_PHI, MAX_RANGE)
    localizer = EKFSLAMVehicleLocalizer(
        true_state=true_state,
        landmark_list=landmark_list,
        landmark_sensor=landmark_sensor,
        init_x=center_x + radius_lm,
        init_y=center_y,
        init_yaw=-pi / 2,
        init_speed_mps=speed,
        sigma_r=SIGMA_R,
        sigma_phi=SIGMA_PHI,
        sigma_v=SIGMA_V,
        sigma_omega=SIGMA_OMEGA,
        gate_threshold=GATE_THRESHOLD,
        max_range=MAX_RANGE,
        duplicate_position_threshold=MIN_LANDMARK_SPACING,
    )

    true_vehicle = FourWheelsVehicle(
        true_state,
        spec,
        controller=controller,
        show_zoom=False,
    )
    est_vehicle = FourWheelsVehicle(
        est_state,
        spec,
        controller=controller,
        sensors=Sensors(),
        localizer=localizer,
        show_zoom=True,
    )
    vis.add_object(true_vehicle)
    vis.add_object(est_vehicle)
    vis.add_object(LegendOverlay())

    if not show_plot:
        vis.not_show_plot()
    vis.draw()

    # After animation ends, show true map vs estimated map comparison
    if show_plot:
        draw_final_comparison(landmark_list, true_state, est_state, localizer)


if __name__ == "__main__":
    main()

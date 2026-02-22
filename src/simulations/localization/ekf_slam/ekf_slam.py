"""
ekf_slam.py

EKF-SLAM (Extended Kalman Filter based Simultaneous Localization and Mapping).
Real-time landmark-based SLAM in 2D with range-bearing sensor.
Demonstrates loop closure via covariance convergence when re-observing landmarks.
"""

import sys
import numpy as np
from pathlib import Path
from math import pi

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"

sys.path.append(abs_dir_path + relative_path + "visualization")
sys.path.append(abs_dir_path + relative_path + "state")
sys.path.append(abs_dir_path + relative_path + "vehicle")
sys.path.append(abs_dir_path + relative_path + "localization/ekf_slam")

import matplotlib.pyplot as plt

from global_xy_visualizer import GlobalXYVisualizer
from min_max import MinMax
from time_parameters import TimeParameters
from vehicle_specification import VehicleSpecification
from state import State
from four_wheels_vehicle import FourWheelsVehicle
from ekf_slam_localizer import EKFSLAMLocalizer
from motion_model import predict_robot_state
from observation_model import observe_landmark, simulate_observation

show_plot = True

SIGMA_R = 0.2
SIGMA_PHI = 0.1
SIGMA_V = 0.15
SIGMA_OMEGA = 0.08
GATE_THRESHOLD = 2.0
MAX_RANGE = 15.0
NUM_LANDMARKS = 18
INTERVAL_SEC = 0.2
# Stop after one lap (circumference / speed); slight overshoot for loop closure
LAP_OVERSHOOT = 1.2


class EKFSLAMSimulation:
    """
    Visualizer object: manages true robot, observations, and delegates EKF
    to EKFSLAMLocalizer. Uses FourWheelsVehicle for estimated-pose rendering.
    """

    def __init__(self, true_landmarks_xy, localizer, est_state, vehicle,
                 true_state=None, true_vehicle=None,
                 sigma_r=SIGMA_R, sigma_phi=SIGMA_PHI,
                 sigma_v=SIGMA_V, sigma_omega=SIGMA_OMEGA,
                 max_range=MAX_RANGE, radius=10.0, speed=1.0,
                 center_x=0.0, center_y=0.0):
        self.true_landmarks = true_landmarks_xy
        self.localizer = localizer
        self.est_state = est_state
        self.vehicle = vehicle
        self.true_state = true_state
        self.true_vehicle = true_vehicle
        self.sigma_r = sigma_r
        self.sigma_phi = sigma_phi
        self.max_range = max_range
        self.radius = radius
        self.speed = speed
        self.Q_input = np.diag([sigma_v ** 2, sigma_omega ** 2])
        self.true_robot = np.array([[center_x + radius], [center_y], [-pi / 2]])
        self.true_trajectory_x = [float(self.true_robot[0, 0])]
        self.true_trajectory_y = [float(self.true_robot[1, 0])]

    def _control(self):
        """Circular path: v constant, omega constant (negative = clockwise)."""
        return np.array([[self.speed], [-self.speed / self.radius]])

    def update(self, dt):
        u = self._control()

        # True motion with process noise
        u_noisy = u + np.sqrt(self.Q_input) @ np.random.randn(2, 1)
        self.true_robot = predict_robot_state(self.true_robot, u_noisy, dt)
        self.true_trajectory_x.append(float(self.true_robot[0, 0]))
        self.true_trajectory_y.append(float(self.true_robot[1, 0]))

        # Push true pose into true vehicle's State for drawing
        if self.true_state is not None:
            self.true_state.update_by_localizer(
                np.array([
                    [self.true_robot[0, 0]],
                    [self.true_robot[1, 0]],
                    [self.true_robot[2, 0]],
                    [self.speed],
                ])
            )

        # EKF prediction
        self.localizer.predict(u, dt)

        # Simulate range-bearing observations from true robot
        observations = []
        observed_indices = []
        for idx, (lx, ly) in enumerate(self.true_landmarks):
            z_pred = observe_landmark(self.true_robot, lx, ly)
            if z_pred[0, 0] <= self.max_range:
                z = simulate_observation(
                    self.true_robot, lx, ly, self.sigma_r, self.sigma_phi
                )
                observations.append(z)
                observed_indices.append(idx)

        # EKF update
        self.localizer.update(observations, observed_indices)

        # Push estimated pose into vehicle's State for drawing
        rs = self.localizer.get_robot_state()
        self.est_state.update_by_localizer(
            np.array([[rs[0, 0]], [rs[1, 0]], [rs[2, 0]], [self.speed]])
        )

    def draw(self, axes, elems):
        # True landmarks
        lx_true = [p[0] for p in self.true_landmarks]
        ly_true = [p[1] for p in self.true_landmarks]
        p1, = axes.plot(lx_true, ly_true, "ko", markersize=8, label="True landmarks")
        elems.append(p1)

        # True trajectory
        p2, = axes.plot(
            self.true_trajectory_x, self.true_trajectory_y,
            "b-", linewidth=1, alpha=0.7, label="True path"
        )
        elems.append(p2)

        # True pose: vehicle body + trajectory (blue)
        if self.true_vehicle is not None:
            self.true_vehicle.draw(axes, elems)

        # Estimated pose: vehicle body + trajectory (green)
        self.vehicle.draw(axes, elems)

        # Covariance ellipse + estimated landmarks via localizer
        self.localizer.draw(axes, elems, self.est_state.x_y_yaw())


def draw_final_comparison(sim):
    """
    After simulation ends, show a side-by-side: true map vs estimated map.
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
    fig.suptitle("EKF-SLAM: True map vs estimated map", fontsize=14)

    # Left: true map
    ax1.set_title("True map (ground truth)")
    ax1.set_aspect("equal")
    ax1.plot(
        [p[0] for p in sim.true_landmarks],
        [p[1] for p in sim.true_landmarks],
        "ko",
        markersize=8,
        label="Landmarks",
    )
    ax1.plot(
        sim.true_trajectory_x,
        sim.true_trajectory_y,
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
    est_lm = sim.localizer.get_estimated_landmarks()
    if est_lm:
        ax2.plot(
            [p[0] for p in est_lm],
            [p[1] for p in est_lm],
            "rx",
            markersize=8,
            label="Est. landmarks",
        )
    ax2.plot(
        sim.est_state.x_history,
        sim.est_state.y_history,
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

    x_lim, y_lim = MinMax(-5, 55), MinMax(-20, 25)
    spec = VehicleSpecification(area_size=20.0, x_lim=x_lim, y_lim=y_lim)

    center_x, center_y = 25.0, 2.5
    radius_lm = 20.0
    speed = 1.0
    # Stop after one lap so the map doesn't keep growing and drift
    lap_time_sec = (2 * pi * radius_lm / speed) * LAP_OVERSHOOT
    time_params = TimeParameters(span_sec=lap_time_sec, interval_sec=INTERVAL_SEC)
    vis = GlobalXYVisualizer(x_lim, y_lim, time_params, show_zoom=True)

    num_outer = 12
    radius_inner = 7.0
    num_inner = 6
    true_landmarks = [
        (center_x + radius_lm * np.cos(2 * pi * i / num_outer),
         center_y + radius_lm * np.sin(2 * pi * i / num_outer))
        for i in range(num_outer)
    ] + [
        (center_x + radius_inner * np.cos(2 * pi * i / num_inner),
         center_y + radius_inner * np.sin(2 * pi * i / num_inner))
        for i in range(num_inner)
    ]

    localizer = EKFSLAMLocalizer(
        init_x=center_x + radius_lm,
        init_y=center_y,
        init_yaw=-pi / 2,
        sigma_r=SIGMA_R,
        sigma_phi=SIGMA_PHI,
        sigma_v=SIGMA_V,
        sigma_omega=SIGMA_OMEGA,
        gate_threshold=GATE_THRESHOLD,
        max_range=MAX_RANGE,
    )

    est_state = State(
        x_m=center_x + radius_lm,
        y_m=center_y,
        yaw_rad=-pi / 2,
        speed_mps=speed,
        color='g',
    )

    true_state = State(
        x_m=center_x + radius_lm,
        y_m=center_y,
        yaw_rad=-pi / 2,
        speed_mps=speed,
        color='b',
    )

    vehicle = FourWheelsVehicle(est_state, spec, show_zoom=True)
    true_vehicle = FourWheelsVehicle(true_state, spec, show_zoom=False)

    sim = EKFSLAMSimulation(
        true_landmarks,
        localizer=localizer,
        est_state=est_state,
        vehicle=vehicle,
        true_state=true_state,
        true_vehicle=true_vehicle,
        sigma_r=SIGMA_R,
        sigma_phi=SIGMA_PHI,
        sigma_v=SIGMA_V,
        sigma_omega=SIGMA_OMEGA,
        max_range=MAX_RANGE,
        radius=radius_lm,
        speed=speed,
        center_x=center_x,
        center_y=center_y,
    )
    vis.add_object(sim)

    if not show_plot:
        vis.not_show_plot()
    vis.draw()

    # After animation ends, show true map vs estimated map comparison
    if show_plot:
        draw_final_comparison(sim)


if __name__ == "__main__":
    main()

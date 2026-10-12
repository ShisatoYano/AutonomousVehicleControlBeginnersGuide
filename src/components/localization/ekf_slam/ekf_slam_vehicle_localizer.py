"""
ekf_slam_vehicle_localizer.py

Adapter that lets EKF-SLAM plug into FourWheelsVehicle like other localizers.
"""

import numpy as np

from ekf_slam_localizer import EKFSLAMLocalizer
from landmark_range_bearing_sensor import LandmarkRangeBearingSensor


class EKFSLAMVehicleLocalizer:
    """
    Adapts EKFSLAMLocalizer to the FourWheelsVehicle localizer interface.
    """

    def __init__(
        self,
        true_state,
        landmark_list,
        landmark_sensor=None,
        init_x=0.0,
        init_y=0.0,
        init_yaw=0.0,
        init_speed_mps=0.0,
        sigma_r=0.2,
        sigma_phi=0.1,
        sigma_v=0.15,
        sigma_omega=0.08,
        gate_threshold=2.0,
        max_range=15.0,
        duplicate_position_threshold=4.5,
        color="r",
    ):
        self.true_state = true_state
        self.landmark_list = landmark_list
        self.landmark_sensor = landmark_sensor or LandmarkRangeBearingSensor(
            sigma_r=sigma_r,
            sigma_phi=sigma_phi,
            max_range_m=max_range,
        )
        self.localizer = EKFSLAMLocalizer(
            init_x=init_x,
            init_y=init_y,
            init_yaw=init_yaw,
            init_speed_mps=init_speed_mps,
            sigma_r=sigma_r,
            sigma_phi=sigma_phi,
            sigma_v=sigma_v,
            sigma_omega=sigma_omega,
            gate_threshold=gate_threshold,
            max_range=max_range,
            duplicate_position_threshold=duplicate_position_threshold,
            color=color,
        )

    def update(self, state, accel_mps2, yaw_rate_rps, time_s, _unused_sensor_data):
        """
        Update the wrapped EKF-SLAM localizer and return the estimated 4D state.
        """
        control = np.array([[accel_mps2], [yaw_rate_rps]])
        self.localizer.predict(control, time_s)

        true_robot = np.array(
            [
                [self.true_state.get_x_m()],
                [self.true_state.get_y_m()],
                [self.true_state.get_yaw_rad()],
                [self.true_state.get_speed_mps()],
            ]
        )
        observations = self.landmark_sensor.observe_visible_landmarks(
            true_robot, self.landmark_list
        )
        self.localizer.update(observations)
        return self.localizer.get_robot_state()

    def draw(self, axes, elems, pose):
        self.localizer.draw(axes, elems, pose)

    def get_estimated_landmarks(self):
        return self.localizer.get_estimated_landmarks()

    def get_robot_state(self):
        return self.localizer.get_robot_state()

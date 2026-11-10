"""
Test of Omni directional LiDAR with a mounting yaw angle

Author: Shisato Yano
"""

from pathlib import Path
import sys

import numpy as np

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/sensors")
sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/sensors/lidar")
sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/state")
sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/obstacle")
from omni_directional_lidar import OmniDirectionalLidar
from sensor_parameters import SensorParameters
from state import State
from obstacle import Obstacle
from obstacle_list import ObstacleList


def _noiseless_points(mount_yaw_deg, vehicle_yaw_deg=0.0):
    # one 2 m x 2 m obstacle in front of the vehicle, its near face at x = 9 m
    obst_list = ObstacleList()
    obst_list.add_obstacle(Obstacle(State(x_m=10.0, y_m=0.0), length_m=1.0, width_m=1.0))

    params = SensorParameters(lon_m=0.0, lat_m=0.0, yaw_deg=mount_yaw_deg,
                              angle_std_scale=0.0, dist_std_rate=0.0)
    lidar = OmniDirectionalLidar(obst_list, params)

    state = State(x_m=0.0, y_m=0.0, yaw_rad=np.deg2rad(vehicle_yaw_deg))
    lidar.install(state)
    lidar.update(state)

    points = np.array([p.get_transformed_data(0.0, 0.0, params.INST_YAW_RAD, 0.0, 0.0, state.get_yaw_rad())
                       for p in lidar.get_point_cloud()])
    return points, params


def test_points_lie_on_obstacle_face_without_mounting_yaw():
    points, _ = _noiseless_points(mount_yaw_deg=0.0)
    assert len(points) > 0
    assert np.allclose(points[:, 0], 9.0, atol=0.05)
    assert np.all(np.abs(points[:, 1]) <= 1.0 + 1e-6)


def test_points_lie_on_obstacle_face_with_mounting_yaw():
    points, _ = _noiseless_points(mount_yaw_deg=30.0)
    assert len(points) > 0
    assert np.allclose(points[:, 0], 9.0, atol=0.05)
    assert np.all(np.abs(points[:, 1]) <= 1.0 + 1e-6)


def test_sensor_odometry_includes_mounting_yaw():
    _, params = _noiseless_points(mount_yaw_deg=30.0, vehicle_yaw_deg=10.0)
    yaw_rad = np.arctan2(params.curr_sensor_tf[1, 0], params.curr_sensor_tf[0, 0])
    assert np.isclose(yaw_rad, np.deg2rad(40.0))

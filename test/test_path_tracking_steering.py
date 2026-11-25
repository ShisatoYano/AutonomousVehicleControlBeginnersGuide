"""
Unit test of the steering of PurePursuitController and StanleyController

Author: abedinm
"""

import sys
from pathlib import Path

import numpy as np
import pytest

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../src/components/"
sys.path.append(abs_dir_path + relative_path + "state")
sys.path.append(abs_dir_path + relative_path + "vehicle")
sys.path.append(abs_dir_path + relative_path + "course/cubic_spline_course")
sys.path.append(abs_dir_path + relative_path + "control/pure_pursuit")
sys.path.append(abs_dir_path + relative_path + "control/stanley")
from state import State
from vehicle_specification import VehicleSpecification
from cubic_spline_course import CubicSplineCourse
from pure_pursuit_controller import PurePursuitController
from stanley_controller import StanleyController

# The vehicle drives along a straight course on the x axis. Each test puts the
# vehicle at a known place, calls update() once and checks the steering angle.
# No plot is drawn.
CONTROLLERS = [PurePursuitController, StanleyController]
TIME_S = 0.1


@pytest.fixture
def straight_course() -> CubicSplineCourse:
    x_points = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
    y_points = [0.0] * len(x_points)
    return CubicSplineCourse(x_points, y_points, 10.0)


def steer_at(controller_class: type, course: CubicSplineCourse, y_m: float, yaw_rad: float = 0.0) -> float:
    """Place the vehicle at (10, y_m) with the given heading and return the steering angle."""
    controller = controller_class(VehicleSpecification(), course)
    state = State(x_m=10.0, y_m=y_m, yaw_rad=yaw_rad, speed_mps=2.0)

    controller.update(state, TIME_S)

    return controller.get_target_steer_rad()


@pytest.mark.parametrize("controller_class", CONTROLLERS)
def test_no_steering_when_on_the_course_and_heading_along_it(controller_class: type, straight_course: CubicSplineCourse) -> None:
    steer_rad = steer_at(controller_class, straight_course, y_m=0.0)

    assert steer_rad == pytest.approx(0.0, abs=1e-3)


@pytest.mark.parametrize("controller_class", CONTROLLERS)
def test_steers_right_when_the_vehicle_is_left_of_the_course(controller_class: type, straight_course: CubicSplineCourse) -> None:
    # The course runs along +x, so y > 0 is the left side. Turning right is a negative angle.
    steer_rad = steer_at(controller_class, straight_course, y_m=1.0)

    assert steer_rad < 0.0


@pytest.mark.parametrize("controller_class", CONTROLLERS)
def test_steers_left_when_the_vehicle_is_right_of_the_course(controller_class: type, straight_course: CubicSplineCourse) -> None:
    steer_rad = steer_at(controller_class, straight_course, y_m=-1.0)

    assert steer_rad > 0.0


@pytest.mark.parametrize("controller_class", CONTROLLERS)
def test_steering_is_mirrored_on_the_two_sides_of_the_course(controller_class: type, straight_course: CubicSplineCourse) -> None:
    left_rad = steer_at(controller_class, straight_course, y_m=1.0)
    right_rad = steer_at(controller_class, straight_course, y_m=-1.0)

    assert left_rad == pytest.approx(-right_rad, abs=1e-3)


@pytest.mark.parametrize("controller_class", CONTROLLERS)
def test_steering_is_larger_when_farther_from_the_course(controller_class: type, straight_course: CubicSplineCourse) -> None:
    near_rad = steer_at(controller_class, straight_course, y_m=0.5)
    far_rad = steer_at(controller_class, straight_course, y_m=2.0)

    assert abs(far_rad) > abs(near_rad)

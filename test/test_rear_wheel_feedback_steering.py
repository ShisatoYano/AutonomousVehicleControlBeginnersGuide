"""
Unit test of the steering of RearWheelFeedbackController

Author: Pham Tran Gia Hung
"""

import sys
from pathlib import Path

import pytest

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../src/components/"
sys.path.append(abs_dir_path + relative_path + "state")
sys.path.append(abs_dir_path + relative_path + "vehicle")
sys.path.append(abs_dir_path + relative_path + "course/cubic_spline_course")
sys.path.append(abs_dir_path + relative_path + "control/rear_wheel_feedback")
from state import State
from vehicle_specification import VehicleSpecification
from cubic_spline_course import CubicSplineCourse
from rear_wheel_feedback_controller import RearWheelFeedbackController

# The vehicle drives along a straight course on the x axis, heading exactly along it
# (heading error 0). Each test puts the vehicle at a known place, calls update() once
# and checks the steering angle. No plot is drawn.
TIME_S = 0.1


@pytest.fixture
def straight_course() -> CubicSplineCourse:
    x_points = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
    y_points = [0.0] * len(x_points)
    return CubicSplineCourse(x_points, y_points, 10.0)


def steer_at(course: CubicSplineCourse, y_m: float) -> float:
    """Place the vehicle at (10, y_m) heading along the course and return the steering angle."""
    controller = RearWheelFeedbackController(VehicleSpecification(), course)
    state = State(x_m=10.0, y_m=y_m, yaw_rad=0.0, speed_mps=5.0)

    controller.update(state, TIME_S)

    return controller.get_target_steer_rad()


def test_steers_right_when_the_vehicle_is_left_of_the_course(straight_course: CubicSplineCourse) -> None:
    # The course runs along +x, so y > 0 is the left side. Turning right is a negative angle.
    steer_rad = steer_at(straight_course, y_m=1.0)

    assert steer_rad < 0.0


def test_steers_left_when_the_vehicle_is_right_of_the_course(straight_course: CubicSplineCourse) -> None:
    steer_rad = steer_at(straight_course, y_m=-1.0)

    assert steer_rad > 0.0


def test_no_steering_when_on_the_course_and_heading_along_it(straight_course: CubicSplineCourse) -> None:
    steer_rad = steer_at(straight_course, y_m=0.0)

    assert steer_rad == pytest.approx(0.0, abs=1e-6)

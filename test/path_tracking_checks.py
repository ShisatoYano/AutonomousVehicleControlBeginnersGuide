"""
path_tracking_checks.py

Author: Adam Amzar

Checks shared by the path tracking simulation tests. Each test passes limits
chosen from the values measured with its own controller, with a margin of about 2x.
"""

import numpy as np


def assert_course_tracked(vehicle, course, max_lateral_error_m: float, max_distance_to_end_m: float,
                          max_final_speed_mps: float = 0.0) -> None:
    """
    Assert that a vehicle followed a course and arrived at its end
    vehicle: Vehicle object after the simulation
    course: Course object the vehicle followed
    max_lateral_error_m: Limit of the largest distance from a vehicle position to the nearest course point[m]
    max_distance_to_end_m: Limit of the distance from the final position to the last course point[m]
    max_final_speed_mps: Limit of the final speed[m/s]. 0.0 means the vehicle has stopped
    """

    course_x_m, course_y_m = np.array(course.x_array), np.array(course.y_array)
    lateral_errors_m = [
        np.min(np.hypot(course_x_m - x_m, course_y_m - y_m))
        for x_m, y_m in zip(vehicle.state.x_history, vehicle.state.y_history)
    ]
    assert max(lateral_errors_m) < max_lateral_error_m

    distance_to_end_m = np.hypot(vehicle.state.get_x_m() - course_x_m[-1], vehicle.state.get_y_m() - course_y_m[-1])
    assert distance_to_end_m < max_distance_to_end_m
    assert vehicle.state.get_speed_mps() <= max_final_speed_mps

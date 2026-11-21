"""
vehicle_part.py

Author: Aikaterini Katsampoula
"""

from typing import Any

import numpy as np
import numpy.typing as npt
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../array")
from xy_array import XYArray


class VehiclePart:
    """
    One drawable part of a vehicle.
    """

    def __init__(
        self,
        spec: Any,
        outline_xy: npt.NDArray[np.float64],
        offset_x_m: float,
        offset_y_m: float,
        steerable: bool = False,
    ) -> None:
        """
        Constructor
        spec: object of VehicleSpecification class
        outline_xy: part outline as a 2 x n array of x and y coordinates in
            meters, in the part's own frame
        offset_x_m: part's x offset from the vehicle origin[m]
        offset_y_m: part's y offset from the vehicle origin[m]
        steerable: whether the part rotates with the steering angle
        """

        self.spec = spec
        self.offset_x_m = offset_x_m
        self.offset_y_m = offset_y_m
        self.steerable = steerable
        self.array = XYArray(np.asarray(outline_xy))

    def draw(
        self,
        axes: Any,
        pose: npt.NDArray[np.float64],
        steer_rad: float,
        elems: list[Any],
    ) -> None:
        """
        Plot the part's outline.
        axes: Axes object of figure
        pose: Vehicle's pose vector as a 3 x 1 array of x and y positions in
            meters and yaw angle in radians
        steer_rad: steering angle[rad]
        elems: List of plot objects
        """
        part_steer_rad = steer_rad if self.steerable else 0.0
        translated_array = self.array.homogeneous_transformation(
            self.offset_x_m, self.offset_y_m, part_steer_rad
        )
        transformed_array = translated_array.homogeneous_transformation(
            pose[0, 0], pose[1, 0], pose[2, 0]
        )
        part_plot, = axes.plot(
            transformed_array.get_x_data(),
            transformed_array.get_y_data(),
            lw=self.spec.line_w,
            color=self.spec.color,
            ls=self.spec.line_type,
        )
        elems.append(part_plot)

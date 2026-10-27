"""
direction_selector.py

Author: Khushi
"""

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../../common")
from angle_lib import pi_to_pi


class DirectionSelector:
    """
    Direction selection class

    This is the third step of the Vector Field Histogram(VFH) algorithm:
    choose one steering direction from the candidate valleys detected in
    Step 2(CandidateValleyDetector), by minimizing a weighted cost function
    over three terms, following Borenstein and Koren(1991):
      - distance from the target(goal) direction
      - distance from the vehicle's current heading(turn effort)
      - distance from the previously selected direction(to avoid
        oscillating back and forth between two similarly-scored valleys)
    All angles handled here are in the global frame. A valley's center
    angle from CandidateValleyDetector is relative to the vehicle's
    heading, so it is converted to a global angle before scoring.
    """

    def __init__(self, target_weight=1.0, heading_weight=1.0, previous_weight=1.0):
        """
        Constructor
        target_weight: Cost weight for distance from the target direction
        heading_weight: Cost weight for distance from the current heading
        previous_weight: Cost weight for distance from the previous selection
        """

        if target_weight < 0.0 or heading_weight < 0.0 or previous_weight < 0.0:
            raise ValueError("weights must be 0 or greater")

        self.target_weight = float(target_weight)
        self.heading_weight = float(heading_weight)
        self.previous_weight = float(previous_weight)
        self.selected_angle_rad = None

    def select(self, valleys, vehicle_yaw_rad, target_angle_rad):
        """
        Function to select one direction(global angle) that minimizes the
        weighted cost function, from the candidate valleys' center angles
        valleys: List of Valley instances from CandidateValleyDetector,
                 whose center angles are relative to the vehicle's heading
        vehicle_yaw_rad: Vehicle's current heading in the global frame[rad]
        target_angle_rad: Direction toward the goal, global frame[rad]
        """

        if not valleys:
            self.selected_angle_rad = None
            return None

        previous_angle_rad = self.selected_angle_rad
        if previous_angle_rad is None:
            previous_angle_rad = vehicle_yaw_rad

        best_angle_rad = None
        lowest_cost = None
        for valley in valleys:
            candidate_angle_rad = pi_to_pi(valley.get_center_angle_rad() + vehicle_yaw_rad)
            cost = self._cost(candidate_angle_rad, target_angle_rad,
                             vehicle_yaw_rad, previous_angle_rad)
            if lowest_cost is None or cost < lowest_cost:
                lowest_cost = cost
                best_angle_rad = candidate_angle_rad

        self.selected_angle_rad = best_angle_rad
        return self.selected_angle_rad

    def _cost(self, candidate_angle_rad, target_angle_rad, vehicle_yaw_rad, previous_angle_rad):
        """
        Private function to calculate one candidate direction's weighted
        cost, all angular differences taken the short way around the circle
        candidate_angle_rad: Candidate direction being scored, global frame[rad]
        target_angle_rad: Direction toward the goal, global frame[rad]
        vehicle_yaw_rad: Vehicle's current heading, global frame[rad]
        previous_angle_rad: Previously selected direction, global frame[rad]
        """

        target_diff = abs(pi_to_pi(candidate_angle_rad - target_angle_rad))
        heading_diff = abs(pi_to_pi(candidate_angle_rad - vehicle_yaw_rad))
        previous_diff = abs(pi_to_pi(candidate_angle_rad - previous_angle_rad))

        return (self.target_weight * target_diff
              + self.heading_weight * heading_diff
              + self.previous_weight * previous_diff)

    def get_selected_angle_rad(self):
        """
        Function to get the most recently selected direction in the global
        frame[rad], or None if no valley has been selected yet(e.g. no
        candidate valleys were available)
        """

        return self.selected_angle_rad

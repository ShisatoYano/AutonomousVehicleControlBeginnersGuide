"""
direction_selector.py

Author: Khushi
"""

import sys
from pathlib import Path

import numpy as np

sys.path.append(str(Path(__file__).absolute().parent) + "/../../common")
from angle_lib import pi_to_pi


class DirectionSelector:
    """
    Direction selection class

    This is the third step of the Vector Field Histogram(VFH) algorithm:
    choose one steering direction from the candidate valleys detected in
    Step 2(CandidateValleyDetector), by minimizing a weighted cost function
    over three terms, following VFH+(Ulrich and Borenstein, 1998):
      - distance from the target(goal) direction
      - distance from the vehicle's current heading(turn effort)
      - distance from the previously selected direction(to avoid
        oscillating back and forth between two similarly-scored valleys)
    All angles handled here are in the global frame. A valley's center
    angle from CandidateValleyDetector is relative to the vehicle's
    heading, so it is converted to a global angle before scoring.

    Each valley offers one or more candidate directions to this cost
    function, following VFH+(Ulrich and Borenstein, 1998): a valley no
    wider than s_max_sectors is "narrow" and offers only its center,
    same as a plain VFH valley would. A wider("wide") valley offers up to
    three candidates instead: s_max_sectors/2 sectors in from each of its
    two borders(so the chosen direction keeps a safety margin from both
    obstacle edges without detouring all the way to a center that may be
    far from anything useful), plus the target direction itself when the
    target lies inside the valley(so a visible, unobstructed goal is
    driven straight at, instead of only ever being offset toward a
    border or a center). See _candidate_angles_rad.
    """

    def __init__(self, target_weight=5.0, heading_weight=2.0, previous_weight=2.0,
                s_max_sectors=5, sector_angle_rad=np.deg2rad(5.0)):
        """
        Constructor
        target_weight: Cost weight for distance from the target direction
        heading_weight: Cost weight for distance from the current heading
        previous_weight: Cost weight for distance from the previous selection.
                         Defaults(5, 2, 2) follow VFH+(Ulrich and Borenstein,
                         1998)'s own example weights, which satisfy
                         target_weight > heading_weight + previous_weight so
                         the vehicle keeps making progress toward the goal
                         instead of favoring smooth turns over reaching it
        s_max_sectors: Width threshold(sectors) distinguishing a "narrow"
                       valley(its center is the only candidate) from a
                       "wide" one(two border-inset candidates, plus the
                       target direction when it falls inside the valley).
                       Should be tuned to the vehicle's width and the
                       polar histogram's sector resolution. Must be 1 or
                       greater
        sector_angle_rad: Angular width of one polar histogram sector[rad],
                          used to place the wide-valley border candidates.
                          Defaults to 72 sectors(5[deg] each), matching
                          PolarHistogram's own default num_sectors. Must
                          be greater than 0
        """

        if target_weight < 0.0 or heading_weight < 0.0 or previous_weight < 0.0:
            raise ValueError("weights must be 0 or greater")
        if s_max_sectors < 1:
            raise ValueError("s_max_sectors must be 1 or greater")
        if sector_angle_rad <= 0.0:
            raise ValueError("sector_angle_rad must be greater than 0")

        self.target_weight = float(target_weight)
        self.heading_weight = float(heading_weight)
        self.previous_weight = float(previous_weight)
        self.s_max_sectors = float(s_max_sectors)
        self.sector_angle_rad = float(sector_angle_rad)
        self.selected_angle_rad = None

    def select(self, valleys, vehicle_yaw_rad, target_angle_rad):
        """
        Function to select one direction(global angle) that minimizes the
        weighted cost function, scored over every candidate direction
        offered by the candidate valleys(see _candidate_angles_rad)
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

        # the target direction is needed relative to the vehicle to check
        # whether it falls inside a valley, since valleys are stored in
        # that same vehicle-relative frame
        target_relative_rad = pi_to_pi(target_angle_rad - vehicle_yaw_rad)

        best_angle_rad = None
        lowest_cost = None
        for valley in valleys:
            for relative_rad in self._candidate_angles_rad(valley, target_relative_rad):
                candidate_angle_rad = pi_to_pi(relative_rad + vehicle_yaw_rad)
                cost = self._cost(candidate_angle_rad, target_angle_rad,
                                 vehicle_yaw_rad, previous_angle_rad)
                if lowest_cost is None or cost < lowest_cost:
                    lowest_cost = cost
                    best_angle_rad = candidate_angle_rad

        self.selected_angle_rad = best_angle_rad
        return self.selected_angle_rad

    def _candidate_angles_rad(self, valley, target_relative_rad):
        """
        Private function to get one valley's vehicle-relative candidate
        direction angle(s)[rad], following VFH+(Ulrich and Borenstein,
        1998). A valley no wider than s_max_sectors offers only its
        center. A wider valley offers its two border-inset candidates
        (s_max_sectors/2 sectors in from each border), plus
        target_relative_rad itself when that direction falls inside the
        valley
        valley: Valley instance being scored
        target_relative_rad: Target direction, relative to the vehicle's
                             current heading[rad]
        """

        half_span_rad = valley.get_width_sectors() * self.sector_angle_rad / 2.0

        if valley.get_width_sectors() <= self.s_max_sectors:
            return [valley.get_center_angle_rad()]

        inset_rad = half_span_rad - (self.s_max_sectors / 2.0) * self.sector_angle_rad
        candidates = [pi_to_pi(valley.get_center_angle_rad() - inset_rad),
                     pi_to_pi(valley.get_center_angle_rad() + inset_rad)]

        target_offset_rad = abs(pi_to_pi(target_relative_rad - valley.get_center_angle_rad()))
        if target_offset_rad <= half_span_rad:
            candidates.append(target_relative_rad)

        return candidates

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

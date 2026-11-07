"""
trajectory_verifier.py

Author: Khushi
"""

from math import hypot


class TrajectoryVerifier:
    """
    Verification class for a VFH+ driven trajectory(Step 6 of the VFH
    roadmap). Each cycle, it reads the same polar histogram a VfhController
    steers by(see PolarHistogramMapper, get_histogram()) and the vehicle's
    current position, to confirm - using the VFH+ pipeline's own sensed
    data, rather than re-deriving obstacle geometry independently - that
    the driven trajectory stays clear of obstacles and reaches its goal.

    A sector's smoothed density rises toward a_gain(default 1.0, see
    PolarHistogram) as a sensed obstacle gets closer, and can exceed it
    when multiple points land in one sector. danger_density is the
    density, anywhere around the vehicle, at/above which this trajectory
    is flagged as a near-collision: too close for comfort even when the
    controller still avoided an actual geometric overlap.
    """

    def __init__(self, mapper, state, target_x_m, target_y_m,
                 danger_density=1.0, goal_tolerance_m=2.0):
        """
        Constructor
        mapper: PolarHistogramMapper instance already driving this vehicle,
                used to read the smoothed density around it each cycle
        state: Vehicle's state object, read each cycle for its position
        target_x_m, target_y_m: Goal point[m] the trajectory should reach
        danger_density: Smoothed density at/above which, anywhere around
                        the vehicle, this cycle counts as a near-collision
        goal_tolerance_m: Distance[m] from the goal counted as reaching it
        """

        if danger_density <= 0.0:
            raise ValueError("danger density must be greater than 0")
        if goal_tolerance_m < 0.0:
            raise ValueError("goal tolerance must be 0 or greater")

        self.mapper = mapper
        self.state = state
        self.target_x_m = target_x_m
        self.target_y_m = target_y_m
        self.danger_density = float(danger_density)
        self.goal_tolerance_m = float(goal_tolerance_m)

        self.max_density_seen = 0.0
        self.near_collision_count = 0

    def update(self, time_s):
        """
        Function to update verification data from the mapper's latest
        histogram
        time_s: Simulation interval time[sec]
        """

        frame_max_density = float(self.mapper.get_histogram().get_smoothed_density().max())

        if frame_max_density > self.max_density_seen:
            self.max_density_seen = frame_max_density

        if frame_max_density >= self.danger_density:
            self.near_collision_count += 1

    def distance_to_goal_m(self):
        """
        Function to get the vehicle's current distance to the goal[m]
        """

        return hypot(self.target_x_m - self.state.get_x_m(),
                    self.target_y_m - self.state.get_y_m())

    def reached_goal(self):
        """
        Function to get whether the vehicle is currently within
        goal_tolerance_m of the goal
        """

        return self.distance_to_goal_m() <= self.goal_tolerance_m

    def get_max_density_seen(self):
        """
        Function to get the highest smoothed density sensed anywhere
        around the vehicle across the whole trajectory so far
        """

        return self.max_density_seen

    def get_near_collision_count(self):
        """
        Function to get the number of simulation cycles where the sensed
        density anywhere around the vehicle reached danger_density
        """

        return self.near_collision_count

    def draw(self, axes, elems):
        """
        Function to draw a small status readout: current distance to
        goal, closest density sensed so far, and a warning once any
        cycle has been a near-collision
        axes: Axes object of figure
        elems: List of plot objects
        """

        near_collision = self.max_density_seen >= self.danger_density
        status = "COLLISION RISK" if near_collision else "clear"
        color = "red" if near_collision else "green"
        text = ("Step 6 verification\n"
               "distance to goal: {0:.1f}[m]\n"
               "closest density seen: {1:.2f}\n"
               "status: {2}").format(self.distance_to_goal_m(), self.max_density_seen, status)

        readout = axes.text(0.02, 0.98, text, transform=axes.transAxes,
                            fontsize=9, va="top", ha="left", color=color,
                            bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))
        elems.append(readout)

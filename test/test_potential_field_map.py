"""
Unit test of PotentialFieldMap

Author: Mugunth007
"""

import sys
import numpy as np
import pytest
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/mapping/potential")
sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/mapping/grid")
from potential_field_map import PotentialFieldMap


def test_attractive_potential_at_goal_and_distance():
    zeta = 0.01
    goal_x, goal_y = 10.0, 20.0
    pf = PotentialFieldMap(zeta=zeta, goal_x_m=goal_x, goal_y_m=goal_y)

    # Potential is 0 at the goal
    assert np.isclose(pf.calculate_attractive_potential(goal_x, goal_y), 0.0)

    # At distance d=5 (dx=3, dy=4 -> d^2 = 25)
    d = 5.0
    expected = 0.5 * zeta * (d ** 2)
    assert np.isclose(pf.calculate_attractive_potential(goal_x + 3.0, goal_y + 4.0), expected)


def test_repulsive_potential_zero_beyond_influence_radius():
    rho = 4.0
    pf = PotentialFieldMap(rho=rho)
    obstacles = [(0.0, 0.0)]

    # Farther than rho (d = 5.0 > 4.0)
    assert np.isclose(pf.calculate_repulsive_potential(5.0, 0.0, obstacles), 0.0)
    assert np.isclose(pf.calculate_repulsive_potential(0.0, -6.0, obstacles), 0.0)


def test_repulsive_potential_increases_closer_to_obstacle():
    rho = 4.0
    pf = PotentialFieldMap(rho=rho, eta=10.0)
    obstacles = [(0.0, 0.0)]

    # Distance 1.0 vs Distance 3.0 inside rho
    u_close = pf.calculate_repulsive_potential(1.0, 0.0, obstacles)
    u_far = pf.calculate_repulsive_potential(3.0, 0.0, obstacles)

    assert u_close > u_far


def test_repulsive_potential_additive_with_multiple_obstacles():
    rho = 4.0
    pf = PotentialFieldMap(rho=rho, eta=10.0)
    # Position (1.0, 0.0) is distance 1.0 from both (0.0, 0.0) and (2.0, 0.0)
    obs_one = [(0.0, 0.0)]
    obs_two = [(0.0, 0.0), (2.0, 0.0)]

    u_one = pf.calculate_repulsive_potential(1.0, 0.0, obs_one)
    u_two = pf.calculate_repulsive_potential(1.0, 0.0, obs_two)

    assert np.isclose(u_two, 2.0 * u_one)

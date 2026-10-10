"""
Test that obstacle yaw angles are converted from degrees to radians

Author: Dhyey Gosa
"""

from pathlib import Path
import numpy as np
import pytest

SIMULATION_DIR = Path(__file__).absolute().parent / "../src/simulations"

# Obstacle yaw angles were written as yaw_rad=np.rad2deg(45), which converts
# 45 radians into about 2578 degrees instead of turning 45 degrees into radians.
# State.yaw_rad expects radians, so the conversion has to be np.deg2rad().
SIMULATION_SCRIPTS = sorted(SIMULATION_DIR.rglob("*.py"))


def _script_with_yaw_angles():
    """
    Function to collect the simulation scripts that convert an angle into the yaw_rad argument
    """
    # Only scripts that feed a numpy conversion into yaw_rad are of interest here.
    # Plain literals such as yaw_rad=0.0 appear in every simulation and are correct.
    return [script for script in SIMULATION_SCRIPTS
            if "yaw_rad=np." in script.read_text(encoding="utf-8")]


def test_simulation_scripts_exist():
    """
    Test to check the simulation scripts were found, otherwise the checks below are meaningless
    """
    assert len(_script_with_yaw_angles()) > 0


def test_yaw_rad_is_never_built_with_rad2deg():
    """
    Test to check no simulation script converts with np.rad2deg() inside a yaw_rad argument
    """
    offenders = [str(script.relative_to(SIMULATION_DIR)) for script in _script_with_yaw_angles()
                 if "yaw_rad=np.rad2deg" in script.read_text(encoding="utf-8")]

    assert offenders == [], f"yaw_rad expects radians, but np.rad2deg() was used in: {offenders}"


def test_yaw_rad_is_built_with_deg2rad():
    """
    Test to check the scripts that set an obstacle yaw angle use np.deg2rad()
    """
    scripts = _script_with_yaw_angles()

    assert len(scripts) > 0
    for script in scripts:
        assert "yaw_rad=np.deg2rad" in script.read_text(encoding="utf-8"), \
            f"{script.relative_to(SIMULATION_DIR)} sets yaw_rad but does not use np.deg2rad()"


@pytest.mark.parametrize("angle_deg", [10.0, 15.0, 45.0, 90.0, 180.0])
def test_deg2rad_yaw_stays_in_radian_range(angle_deg):
    """
    Test to check a degree input converted with np.deg2rad() stays inside the valid radian range
    """
    yaw_rad = np.deg2rad(angle_deg)

    assert -np.pi <= yaw_rad <= np.pi
    assert yaw_rad != np.rad2deg(angle_deg)


def test_rad2deg_would_be_out_of_range():
    """
    Test to check the old np.rad2deg() call really produced an out of range radian value
    """
    assert not (-np.pi <= np.rad2deg(45.0) <= np.pi)
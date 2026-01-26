"""
Test of Potential field map construction

Author: Panav Arpit Raaj
"""

from pathlib import Path
import sys
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/mapping/potential_field_map_construction")
import potential_field_map_construction


def test_simulation():
    potential_field_map_construction.show_plot = False

    potential_field_map_construction.main()

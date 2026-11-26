"""
Test of Dynamic Window Approach simulation

Author: Yuvraj Gupta
"""

from pathlib import Path
import sys
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/path_planning/dynamic_window_approach")
import dynamic_window_approach


def test_simulation():
    dynamic_window_approach.show_plot = False

    dynamic_window_approach.main()

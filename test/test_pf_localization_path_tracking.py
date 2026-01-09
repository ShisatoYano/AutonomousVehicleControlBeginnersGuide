"""
Test of Localization and Path tracking Simulation by Particle Filter and Pure pursuit

Author: Sahruday Patti
"""

from pathlib import Path
import sys
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/localization/particle_filter_localization")
import particle_filter_localization


def test_simulation():
    particle_filter_localization.show_plot = False

    particle_filter_localization.main()


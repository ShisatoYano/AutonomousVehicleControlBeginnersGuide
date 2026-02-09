"""
Test of Elastic Bands path planning and navigation simulation

Author: Erwin Lejeune
"""

from pathlib import Path
import sys
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/path_planning/elastic_bands_path_planning")
import elastic_bands_path_planning


def test_simulation():
    elastic_bands_path_planning.show_plot = False

    elastic_bands_path_planning.main()

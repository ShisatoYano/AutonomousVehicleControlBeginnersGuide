"""
Test of Elastic Bands path planning and navigation simulation

Author: Erwin Lejeune
"""

from pathlib import Path
import sys
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/path_planning/elastic_bands_path_planning")
import elastic_bands_path_planning


def test_simulation(tmp_path):
    elastic_bands_path_planning.show_plot = False
    elastic_bands_path_planning.output_dir = str(tmp_path)

    elastic_bands_path_planning.main()

    assert (tmp_path / "map.json").exists()
    assert (tmp_path / "path.json").exists()

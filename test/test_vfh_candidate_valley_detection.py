"""
Test of VFH candidate valley detection simulation

Author: Khushi
"""

from pathlib import Path
import sys

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/mapping/vfh_candidate_valley_detection")
import vfh_candidate_valley_detection


def test_simulation():
    vfh_candidate_valley_detection.show_plot = False

    vfh_candidate_valley_detection.main()

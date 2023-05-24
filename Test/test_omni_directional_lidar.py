"""
Unit test of OmniDirectionalLidar

Author: Shisato Yano
"""

import matplotlib as mpl
mpl.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pytest
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/sensors/lidar")
from omni_directional_lidar import OmniDirectionalLidar


# mock classes
class MockObstacle:
    def __init__(self):
        pass

    def vertex_xy(self):
        return [0.0, 1.0, 2.0, 3.0, 4.0, 5.0], [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]

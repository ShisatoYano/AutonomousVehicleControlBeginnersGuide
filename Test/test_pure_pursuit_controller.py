"""
Unit test of PurePursuitController

Author: Shisato Yano
"""

import matplotlib as mpl
mpl.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from math import sin
import pytest
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../src/control/pure_pursuit")
from pure_pursuit_controller import PurePursuitController


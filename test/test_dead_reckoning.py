"""
Test code of Localization by Dead Reckoning

Author: Shisato Yano
"""

import pytest
import sys
import os

# テストターゲットのモジュールを読み込むためのパス設定
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../vehicle_drawing")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../common")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../motion_model/linear_motion")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../localization/dead_reckoning")
import dead_reckoning as dr


def test_simulation():
    dr.show_plot = False
    
    assert dr.main() == True

"""
Test code of Linear motion model

Author: Shisato Yano
"""

import pytest
import sys
import os

# テストターゲットのモジュールを読み込むためのパス設定
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../vehicle_drawing")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../common")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../motion_model/linear_motion")
import linear_motion_model as lmm


def test_simulation():
    lmm.show_plot = False
    
    assert lmm.main() == True

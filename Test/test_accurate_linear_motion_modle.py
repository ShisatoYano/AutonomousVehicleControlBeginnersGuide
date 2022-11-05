"""
Test code of Accurate linear motion model

Author: Shisato Yano
"""

import pytest
import sys
import os

# テストターゲットのモジュールを読み込むためのパス設定
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../Modeling/Sources/motion")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../Modeling/Sources/vehicle")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../Basis/Sources/transformation")
import accurate_linear_motion_model as almm


def test_simulation():
    almm.show_plot = False
    
    assert almm.main() == True

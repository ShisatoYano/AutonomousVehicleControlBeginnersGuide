"""
Test code of Comparing linear motion model

Author: Shisato Yano
"""

import pytest
import sys
import os

# テストターゲットのモジュールを読み込むためのパス設定
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../Modeling/Sources/vehicle")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../common")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../motion_model/linear_motion")
import compare_linear_model as clm


def test_simulation():
    clm.show_plot = False
    
    assert clm.main() == True

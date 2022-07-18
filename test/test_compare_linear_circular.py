"""
Test code of Comparing linear vs circular motion model

Author: Shisato Yano
"""

import pytest
import sys
import os

# テストターゲットのモジュールを読み込むためのパス設定
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../vehicle_drawing")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../common")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../motion_model/linear_motion")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../motion_model/circular_motion")
import compare_linear_circular as clc


def test_simulation():
    clc.show_plot = False
    
    assert clc.main() == True

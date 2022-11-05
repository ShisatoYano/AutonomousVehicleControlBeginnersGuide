"""
Test code of Two wheels vehicle drawing

Author: Shisato Yano
"""

import pytest
import sys
import os

# テストターゲットのモジュールを読み込むためのパス設定
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../Modeling/Sources/vehicle")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../Basis/Sources/transformation")
import two_wheels_vehicle


def test_draw():
    two_wheels_vehicle.show_plot = False
    
    assert two_wheels_vehicle.main() == True
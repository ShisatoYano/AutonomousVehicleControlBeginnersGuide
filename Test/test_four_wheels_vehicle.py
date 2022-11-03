"""
Test code of Four wheels vehicle drawing

Author: Shisato Yano
"""

import pytest
import sys
import os

# テストターゲットのモジュールを読み込むためのパス設定
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../Modeling/Sources/vehicle")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../common")
import four_wheels_vehicle


def test_draw():
    four_wheels_vehicle.show_plot = False
    
    assert four_wheels_vehicle.main() == True
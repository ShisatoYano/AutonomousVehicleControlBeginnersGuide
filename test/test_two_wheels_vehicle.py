"""
Test code of Two wheels vehicle drawing

Author: Shisato Yano
"""

import pytest
import sys
import os

# テストターゲットのモジュールを読み込むためのパス設定
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../vehicle_drawing")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../common")
import two_wheels_vehicle


def test_draw():
    two_wheels_vehicle.show_plot = False
    
    assert two_wheels_vehicle.main() == True

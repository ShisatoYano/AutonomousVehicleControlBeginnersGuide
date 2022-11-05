"""
Test code of Vehicle chassis drawing

Author: Shisato Yano
"""

import pytest
import sys
import os

# テストターゲットのモジュールを読み込むためのパス設定
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../Modeling/Sources/vehicle")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../Basis/Sources/transformation")
import chassis


def test_draw():
    chassis.show_plot = False
    
    assert chassis.main() == True

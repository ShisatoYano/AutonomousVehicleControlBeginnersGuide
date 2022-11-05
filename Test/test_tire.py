"""
Test code of Vehicle tire drawing

Author: Shisato Yano
"""

import pytest
import sys
import os

# テストターゲットのモジュールを読み込むためのパス設定
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../Modeling/Sources/vehicle")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../Basis/Sources/transformation")
import tire


def test_draw():
    tire.show_plot = False
    
    assert tire.main() == True

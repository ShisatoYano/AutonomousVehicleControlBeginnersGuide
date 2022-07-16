"""
Test code of Vehicle tire drawing

Author: Shisato Yano
"""

import pytest
import sys
import os

# テストターゲットのモジュールを読み込むためのパス設定
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../vehicle_drawing")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../common")
import tire


def test_draw():
    tire.show_plot = False
    
    assert tire.main() == True

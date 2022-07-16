"""
Test code of Vehicle body drawing

Author: Shisato Yano
"""

import pytest
import sys
import os

# テストターゲットのモジュールを読み込むためのパス設定
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../vehicle_drawing")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../common")
import body


def test_main():
    body.show_plot = False
    
    assert body.main() == True

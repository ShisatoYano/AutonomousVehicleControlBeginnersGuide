"""
Test code of Circular motion model

Author: Shisato Yano
"""

import pytest
import sys
import os

# テストターゲットのモジュールを読み込むためのパス設定
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../vehicle_drawing")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../common")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../motion_model/circular_motion")
import circular_motion_model as cmm


def test_simulation():
    cmm.show_plot = False
    
    assert cmm.main() == True

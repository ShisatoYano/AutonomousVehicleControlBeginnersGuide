"""
Test code of GNSS observation

Author: Shisato Yano
"""

import pytest
import sys
import os

# テストターゲットのモジュールを読み込むためのパス設定
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../vehicle_drawing")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../common")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../motion_model/linear_motion")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../sensor_model/gnss_model")
import gnss_model as gnss


def test_simulation():
    gnss.show_plot = False

    assert gnss.main() == True
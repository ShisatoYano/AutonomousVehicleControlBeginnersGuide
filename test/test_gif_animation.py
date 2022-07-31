"""
Test code of Gif animation creation

Author: Shisato Yano
"""

import pytest
import sys
import os
import numpy as np
import matplotlib.pyplot as plt

# テストターゲットのモジュールを読み込むためのパス設定
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../common")
from gif_animation import GifAnimation


def test_gif_creation():
    save_name_path = os.path.dirname(os.path.abspath(__file__)) + "/../gif/test.gif"

    ga = GifAnimation(save_name_path=save_name_path, duration_ms=100)

    for i in range(10):
        x = np.arange(0, 10, 0.01)
        y = np.sin(2 * np.pi * 1 * x - i)
        plt.plot(x, y)

        ga.save_image()
    
    ga.create_gif()

    assert os.path.isfile(save_name_path) == True

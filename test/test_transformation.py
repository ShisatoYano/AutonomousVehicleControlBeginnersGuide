"""
Test code of Transformation

Author: Shisato Yano
"""

import pytest
import sys
import os
import numpy as np

# テストターゲットのモジュールを読み込むためのパス設定
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../common")
import transformation as tf


def test_rotate_translate_2d():
    original_point = np.array([[1.0], [0.0]])
    transformed_point = tf.rotate_translate_2d(original_point, 1.0, 1.0, np.deg2rad(90))
    assert transformed_point[0, 0] == 1.0
    assert transformed_point[1, 0] == 2.0


def test_limit_angle_pi_2_pi():
    assert tf.limit_angle_pi_2_pi(np.deg2rad(180)) == round(np.deg2rad(-180), 2)
    assert tf.limit_angle_pi_2_pi(np.deg2rad(225)) == round(np.deg2rad(-135), 2)
    assert tf.limit_angle_pi_2_pi(np.deg2rad(270)) == round(np.deg2rad(-90), 2)
    assert tf.limit_angle_pi_2_pi(np.deg2rad(300)) == round(np.deg2rad(-60), 2)


def test_convert_speed_kmh_2_ms():
    assert tf.convert_speed_kmh_2_ms(10) == 2.78
    assert tf.convert_speed_kmh_2_ms(30) == 8.33
    assert tf.convert_speed_kmh_2_ms(50) == 13.89

"""
Transformation function library

Author: Shisato Yano
"""

from math import sin, cos, pi
import numpy as np


def rotate_translate_2d(points, x_m, y_m, angle_rad):
    """
    2次元点群を座標変換する関数
    angle_degだけ回転させて、x_m, y_mだけ平行移動
    points: 2 x 点数の2次元配列, np.array([[x1, x2, x3], [y1, y2, y3]])など
    x_m: X軸方向の平行移動量
    y_m: Y軸方向の平行移動量
    angle_rad: 回転角度[rad]
    """
    
    # 回転角度の単位をradにして、sin, cos成分を計算
    angle_cos = cos(angle_rad)
    angle_sin = sin(angle_rad)

    # 回転行列を定義
    rotation_matrix = np.array([[angle_cos, -angle_sin], [angle_sin, angle_cos]])
    
    # 入力点群を回転
    rotated_points = rotation_matrix.dot(points)
    
    # 回転させた点群を平行移動
    transformed_points = rotated_points + np.ones(points.shape) * (np.array([[x_m], [y_m]]))
    
    return transformed_points # 回転 + 平行移動させた点群


def limit_angle_pi_2_pi(angle_rad):
    """
    角度値を-180°～180°に制限する関数
    angle_rad: 入力角度[rad]
    """
    
    return round((angle_rad + pi) % (2 * pi) - pi, 2) # 制限したあとの角度値


def convert_speed_kmh_2_ms(speed_kmh):
    """
    速度の単位をkm/hからm/sに変換する関数
    speed_kmh: 入力速度[km/h]
    """
    
    return round(speed_kmh / 3.6, 2) # m/sに変換した後の速度

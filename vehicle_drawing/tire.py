"""
Vehicle tire drawing program

Author: Shisato Yano
"""

import numpy as np
import matplotlib.pyplot as plt
import sys

# 他のディレクトリにあるモジュールを読み込むためのパス設定
sys.path.append("../common")
from transformation import rotate_translate_2d


class Tire:
    """
    車両のタイヤを描画するクラス
    """
    
    def __init__(self, axes, radius_m, width_half_m, offset_x_m, offset_y_m):
        """
        コンストラクタ
        axes: 描画オブジェクト
        radius_m: タイヤ半径[m]
        width_half_m: 車体半分の横幅[m]
        offset_x_m: 車両位置から縦方向への取付位置[m]
        offset_y_m: 車両位置から横方向への取付位置[m]
        """
        
        # パラメータのセット
        self.radius = radius_m
        self.width_half = width_half_m
        self.offset_x = offset_x_m
        self.offset_y = offset_y_m

        # タイヤの形を形成するための点群
        self.points = np.array([
            [self.radius, -self.radius, -self.radius, self.radius, self.radius],
            [self.width_half, self.width_half, -self.width_half, -self.width_half, self.width_half]
        ])

        # 描画オブジェクトの初期化
        self.plot, = axes.plot(self.points[0, :], self.points[1, :], lw=1, color='k')
    
    def draw(self, x_m, y_m, yaw_angle_deg, steering_angle_deg):
        """
        タイヤの形を描画する関数
        指定した分だけ回転 + 並進移動させて描画する
        x_m: X軸方向の並進移動量
        y_m: Y軸方向の並進移動量
        yaw_angle_deg: 車両の方位角度[deg]
        steering_angle_deg: ステアリング角度[deg]
        """

        # ステアリング角度分だけ向きを変える
        # 車体上での取り付け位置分だけオフセット
        transformed_points = rotate_translate_2d(self.points, self.offset_x, self.offset_y, np.deg2rad(steering_angle_deg))
        
        # 車両の方位角度分だけ回転
        # 車両の移動量分だけ並進移動
        transformed_points = rotate_translate_2d(transformed_points, x_m, y_m, np.deg2rad(yaw_angle_deg))
        
        # 描画
        self.plot.set_data(transformed_points[0, :], transformed_points[1, :])

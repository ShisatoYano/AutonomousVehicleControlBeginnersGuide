"""
Vehicle axle drawing program

Author: Shisato Yano
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
from os.path import dirname, abspath

# 他のディレクトリにあるモジュールを読み込むためのパス設定
parent_dir = dirname(dirname(abspath(__file__)))
if parent_dir not in sys.path: sys.path.append(parent_dir)
from common.transformation import rotate_translate_2d


class Axle:
    """
    アクスルを描画するクラス
    """

    def __init__(self, axes, offset_x_m, offset_y_m):
        """
        コンストラクタ
        axes: 描画オブジェクト
        offset_x_m: 車両位置から前方への取付位置[m]
        offset_y_m: 車両位置から後方への取付位置[m]
        """

        # パラメータのセット
        self.offset_x = offset_x_m
        self.offset_y = offset_y_m

        # アクスルの形を形成するための点群
        self.points = np.array([
            [0.0, 0.0],
            [self.offset_y, -self.offset_y]
        ])

        # 描画オブジェクトの初期化
        self.plot, = axes.plot(self.points[0, :], self.points[1, :], lw=1, color='k')
    
    def draw(self, x_m, y_m, yaw_angle_deg, steer_angle_deg):
        """
        アクスルの形を描画する関数
        指定した分だけ回転 + 並進移動させて描画する
        x_m: X軸方向の並進移動量
        y_m: Y軸方向の並進移動量
        yaw_angle_deg: 車両の方位角度[deg]
        steering_angle_deg: ステアリング角度[deg]
        """

        # ステアリング角度分だけ向きを変える
        # 車体上での取り付け位置分だけオフセット
        transformed_points = rotate_translate_2d(self.points, self.offset_x, 0.0, np.deg2rad(steer_angle_deg))
        
        # 車両の方位角度分だけ回転
        # 車両の移動量分だけ並進移動
        transformed_points = rotate_translate_2d(transformed_points, x_m, y_m, np.deg2rad(yaw_angle_deg))
        
        # 描画
        self.plot.set_data(transformed_points[0, :], transformed_points[1, :])

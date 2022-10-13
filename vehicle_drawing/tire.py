"""
Vehicle tire drawing program

Author: Shisato Yano
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import os

# 他のディレクトリにあるモジュールを読み込むためのパス設定
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../common")
from transformation import rotate_translate_2d

# グラフの出力有無を切り替えるフラグ
show_plot = True


class Tire:
    """
    車両のタイヤを描画するクラス
    """
    
    def __init__(self, axes, radius_m, width_half_m, offset_x_m, offset_y_m, color, line_width, line_type):
        """
        コンストラクタ
        axes: 描画オブジェクト
        radius_m: タイヤ半径[m]
        width_half_m: 車体半分の横幅[m]
        offset_x_m: 車両位置から縦方向への取付位置[m]
        offset_y_m: 車両位置から横方向への取付位置[m]
        color: ラインの色
        line_width: ラインの幅
        line_type: ラインの種類
        """
        
        # パラメータのセット
        self.radius = radius_m
        self.width_half = width_half_m
        self.offset_x = offset_x_m
        self.offset_y = offset_y_m
        self.color = color
        self.line_width = line_width
        self.line_type = line_type
        
        # タイヤの形を形成するための点群
        self.points = np.array([
            [self.radius, -self.radius, -self.radius, self.radius, self.radius],
            [self.width_half, self.width_half, -self.width_half, -self.width_half, self.width_half]
        ])

        # 描画オブジェクトの初期化
        self.plot, = axes.plot(self.points[0, :], self.points[1, :], lw=self.line_width, color=self.color, ls=self.line_type)
    
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
    
    def set_color(self, color):
        """
        Function to set color parameter
        How to use:
        tire_obj = Tire(input_1, input_2, ...)
        tire_obj.set_color('r') or tire_obj.set_color("#F08080")
        color: char or color code of color you want to set
        """

        self.color = color # set into member variable
        self.plot.set_color(self.color) # update color setting of plot object

    def set_color(self, color):
        self.color = color
        self.plot.set_color(self.color)


def main():
    """
    このファイルを単体で実行したときの処理を実装したメイン関数
    車両のタイヤの絵を描画する
    """

    print(__file__ + "start!!")

    # 既に作成されている図があればクリア
    plt.clf()

    # 描画の設定
    ax = plt.subplot(1, 1, 1)
    ax.set_xlabel("X[m]")
    ax.set_ylabel("Y[m]")
    ax.set_xlim([-10, 10])
    ax.set_ylim([-10, 10])
    ax.set_aspect("equal")
    ax.grid(True)

    # 描画クラスのインスタンス生成
    tire = Tire(ax, 1.27, 0.64, 0.0, 0.0, 'k', 1.0, '-')

    # 描画
    tire.draw(0.0, 0.0, 0.0, 0.0)

    # ユニットテスト時はこのフラグをFlaseにする
    # グラフが表示されるとテストが進まなくなる
    if show_plot: plt.show()

    return True


# メイン関数の実行
if __name__ == "__main__":
    main()

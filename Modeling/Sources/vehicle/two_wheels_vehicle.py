"""
Two wheels vehicle drawing program

Author: Shisato Yano
"""

import matplotlib.pyplot as plt
import sys
import os

from body import Body
from tire import Tire
from chassis import Chassis

# グラフの出力有無を切り替えるフラグ
show_plot = True


class TwoWheelsVehicle:
    """
    2輪モデルの車両を描画するクラス
    """
    
    def __init__(self, axes, front_length_m=2.0, rear_length_m=0.0, 
                 tire_radius_m=0.3, tire_width_half_m=0.12, color='k', line_width=1.0, line_type='-'):
        """
        コンストラクタ
        axes: 描画オブジェクト
        front_length_m: 車両位置から前方への長さ[m]
        rear_length_m: 車両位置から後方への長さ[m]
        tire_radius_m: タイヤ半径[m]
        tire_width_half_m: タイヤ幅の半分[m]
        color: ラインの色
        """
        
        # 各パーツクラスのインスタンス生成
        self.body = Body(axes, front_length_m, rear_length_m, color, line_width, line_type)
        self.front_tire = Tire(axes, tire_radius_m, tire_width_half_m, front_length_m, 0.0, color, line_width, line_type)
        self.rear_tire = Tire(axes, tire_radius_m, tire_width_half_m, -rear_length_m, 0.0, color, line_width, line_type)
        self.chassis = Chassis(axes, front_length_m, rear_length_m, color, line_width, line_type)
    
    def draw(self, x_m, y_m, yaw_angle_deg, steer_angle_deg):
        """
        車両の形を描画する関数
        指定した分だけ回転 + 並進移動させて描画する
        x_m: X軸方向の並進移動量
        y_m: Y軸方向の並進移動量
        yaw_angle_deg: 車両の方位角度[deg]
        steering_angle_deg: ステアリング角度[deg]
        """

        # 各パーツインスタンスの描画メソッドを呼び出す
        self.body.draw(x_m, y_m, yaw_angle_deg)
        self.front_tire.draw(x_m, y_m, yaw_angle_deg, steer_angle_deg)
        self.rear_tire.draw(x_m, y_m, yaw_angle_deg, 0.0)
        self.chassis.draw(x_m, y_m, yaw_angle_deg)


def main():
    print(__file__ + " start!!")

    # 既に作成されている図があればクリア
    plt.clf()

    # 描画の設定
    ax = plt.subplot(1, 1, 1)
    ax.set_xlabel("X[m]")
    ax.set_ylabel("Y[m]")
    ax.set_xlim([-1, 3])
    ax.set_ylim([-2, 2])
    ax.set_aspect("equal")
    ax.grid(True)

    # 描画クラスのインスタンス生成
    twv = TwoWheelsVehicle(ax)
    
    twv.draw(0, 0, 0, 0)

    if show_plot: plt.show()

    return True


# メイン関数の実行
if __name__ == "__main__":
    main()
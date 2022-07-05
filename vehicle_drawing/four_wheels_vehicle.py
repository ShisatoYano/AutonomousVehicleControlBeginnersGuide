"""
Four wheels vehicle drawing program

Author: Shisato Yano
"""

import matplotlib.pyplot as plt

from body import Body
from tire import Tire
from chassis import Chassis
from axle import Axle


class FourWheelsVehicle:
    """
    4輪モデルの車両を描画するクラス
    """

    def __init__(self, axes, front_length_m=6.35, rear_length_m=0.0, tire_radius_m=1.27, tire_width_half_m=0.64, axle_half_m=2.54):
        """
        コンストラクタ
        axes: 描画オブジェクト
        front_length_m: 車両位置から前方への長さ[m]
        rear_length_m: 車両位置から後方への長さ[m]
        tire_radius_m: タイヤ半径[m]
        tire_width_half_m: タイヤ幅の半分[m]
        axle_half_m: アクスル長さの半分[m]
        """

        # 各パーツクラスのインスタンス生成
        self.body = Body(axes, front_length_m, rear_length_m)
        self.chassis = Chassis(axes, front_length_m, rear_length_m)
        self.front_axle = Axle(axes, front_length_m, axle_half_m)
        self.rear_axle = Axle(axes, -rear_length_m, axle_half_m)
        self.front_left_tire = Tire(axes, tire_radius_m, tire_width_half_m, front_length_m, axle_half_m)
        self.front_right_tire = Tire(axes, tire_radius_m, tire_width_half_m, front_length_m, -axle_half_m)
        self.rear_left_tire = Tire(axes, tire_radius_m, tire_width_half_m, -rear_length_m, axle_half_m)
        self.rear_right_tire = Tire(axes, tire_radius_m, tire_width_half_m, -rear_length_m, -axle_half_m)
    
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
        self.chassis.draw(x_m, y_m, yaw_angle_deg)
        self.front_axle.draw(x_m, y_m, yaw_angle_deg, 0.0)
        self.rear_axle.draw(x_m, y_m, yaw_angle_deg, 0.0)
        self.front_left_tire.draw(x_m, y_m, yaw_angle_deg, steer_angle_deg)
        self.front_right_tire.draw(x_m, y_m, yaw_angle_deg, steer_angle_deg)
        self.rear_left_tire.draw(x_m, y_m, yaw_angle_deg, 0.0)
        self.rear_right_tire.draw(x_m, y_m, yaw_angle_deg, 0.0)


# メイン処理
# このファイルを実行すると、方位の向きと
# ステアリング角度を変えた車両の絵が描画される
if __name__ == "__main__":
    print(__file__ + " start!!")

    # 描画の設定
    ax = plt.subplot(1, 1, 1)
    ax.set_xlabel("X[m]")
    ax.set_ylabel("Y[m]")
    ax.set_xlim([-10, 10])
    ax.set_ylim([-10, 10])
    ax.set_aspect("equal")
    ax.grid(True)

    # 描画クラスのインスタンス生成
    fwv = FourWheelsVehicle(ax)

    # 角度を30°ずつずらしながら描画
    angle_deg = -60
    while angle_deg <= 60:
        fwv.draw(0, 0, angle_deg, angle_deg) # 描画メソッドの実行

        angle_deg += 30 # 30度ずらす
        
        plt.pause(2) # 一度描画するたびに2秒ポーズ

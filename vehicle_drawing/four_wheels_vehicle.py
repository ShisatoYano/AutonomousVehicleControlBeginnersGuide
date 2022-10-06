"""
Four wheels vehicle drawing program

Author: Shisato Yano
"""

import matplotlib.pyplot as plt
import sys
import os

# 他のディレクトリにあるモジュールを読み込むためのパス設定
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../common")
from body import Body
from tire import Tire
from chassis import Chassis
from axle import Axle
from gif_animation import GifAnimation

# グラフの出力有無を切り替えるフラグ
show_plot = True


class FourWheelsVehicle:
    """
    4輪モデルの車両を描画するクラス
    """

    def __init__(self, axes, front_length_m=2.0, rear_length_m=0.0, tire_radius_m=0.34, 
                 tire_width_half_m=0.2, axle_half_m=0.75, color='k', line_width=1.0, line_type='-'):
        """
        コンストラクタ
        axes: 描画オブジェクト
        front_length_m: 車両位置から前方への長さ[m]
        rear_length_m: 車両位置から後方への長さ[m]
        tire_radius_m: タイヤ半径[m]
        tire_width_half_m: タイヤ幅の半分[m]
        axle_half_m: アクスル長さの半分[m]
        color: ラインの色
        line_width: ラインの幅
        line_type: ラインの種類
        """

        # 各パーツクラスのインスタンス生成
        self.body = Body(axes, front_length_m, rear_length_m, color, line_width, line_type)
        self.chassis = Chassis(axes, front_length_m, rear_length_m, color, line_width, line_type)
        self.front_axle = Axle(axes, front_length_m, axle_half_m, color, line_width, line_type)
        self.rear_axle = Axle(axes, -rear_length_m, axle_half_m, color, line_width, line_type)
        self.front_left_tire = Tire(axes, tire_radius_m, tire_width_half_m, front_length_m, axle_half_m, color, line_width, line_type)
        self.front_right_tire = Tire(axes, tire_radius_m, tire_width_half_m, front_length_m, -axle_half_m, color, line_width, line_type)
        self.rear_left_tire = Tire(axes, tire_radius_m, tire_width_half_m, -rear_length_m, axle_half_m, color, line_width, line_type)
        self.rear_right_tire = Tire(axes, tire_radius_m, tire_width_half_m, -rear_length_m, -axle_half_m, color, line_width, line_type)
    
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

    def set_tire_color(self, color):
        """
        Function to set color parameter of tire
        How to use:
        fwv = FourWheelsVehicle(input_1, input_2, ...)
        fwv.set_tire_color('r') or fwv.set_tire_color("#F08080")
        color: char or color code of color you want to set
        """

        self.front_left_tire.set_color(color)
        self.front_right_tire.set_color(color)
        self.rear_left_tire.set_color(color)
        self.rear_right_tire.set_color(color)

def main():
    print(__file__ + " start!!")

    # 既に作成されている図があればクリア
    plt.clf()

    # 描画の設定
    ax = plt.subplot(1, 1, 1)
    ax.set_xlabel("X[m]")
    ax.set_ylabel("Y[m]")
    ax.set_xlim([-5, 5])
    ax.set_ylim([-5, 5])
    ax.set_aspect("equal")
    ax.grid(True)

    # 描画クラスのインスタンス生成
    fwv = FourWheelsVehicle(ax)
    fwv.set_tire_color('r')

    # Gif作成クラスのインスタンス生成
    save_name_path = os.path.dirname(os.path.abspath(__file__)) + "/../gif/colerful_four_wheels_vehicle.gif"
    ga = GifAnimation(save_name_path=save_name_path, duration_ms=2000)

    # 角度を30°ずつずらしながら描画
    angle_deg = -60
    while angle_deg <= 60:
        fwv.draw(0, 0, angle_deg, angle_deg) # 描画メソッドの実行

        angle_deg += 30 # 30度ずらす
        
        # ユニットテスト時はこのフラグをFlaseにする
        # グラフが表示されるとテストが進まなくなる
        if show_plot:
            ga.save_image() # 描画した図を画像として保存
            plt.pause(2) # 一度描画するたびに2秒ポーズ
    
    ga.create_gif() # 保存した画像を繋ぎ合わせてGifを作成
    
    return True


# メイン関数の実行
if __name__ == "__main__":
    main()

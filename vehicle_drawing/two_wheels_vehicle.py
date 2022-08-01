"""
Two wheels vehicle drawing program

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
from gif_animation import GifAnimation

# グラフの出力有無を切り替えるフラグ
show_plot = True


class TwoWheelsVehicle:
    """
    2輪モデルの車両を描画するクラス
    """
    
    def __init__(self, axes, front_length_m=6.35, rear_length_m=0.0, 
                 tire_radius_m=1.27, tire_width_half_m=0.64, color='k'):
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
        self.body = Body(axes, front_length_m, rear_length_m, color)
        self.front_tire = Tire(axes, tire_radius_m, tire_width_half_m, front_length_m, 0.0, color)
        self.rear_tire = Tire(axes, tire_radius_m, tire_width_half_m, -rear_length_m, 0.0, color)
        self.chassis = Chassis(axes, front_length_m, rear_length_m, color)
    
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
    ax.set_xlim([-10, 10])
    ax.set_ylim([-10, 10])
    ax.set_aspect("equal")
    ax.grid(True)

    # 描画クラスのインスタンス生成
    twv = TwoWheelsVehicle(ax)

    # Gif作成クラスのインスタンス生成
    save_name_path = os.path.dirname(os.path.abspath(__file__)) + "/../gif/two_wheels_vehicle.gif"
    ga = GifAnimation(save_name_path=save_name_path, duration_ms=2000)

    # 角度を30°ずつずらしながら描画
    angle_deg = -60
    while angle_deg <= 60:
        twv.draw(0, 0, angle_deg, angle_deg) # 描画メソッドの実行

        angle_deg += 30 # 30度ずらす

        ga.save_image() # 描画した図を画像として保存
        
        # ユニットテスト時はこのフラグをFlaseにする
        # グラフが表示されるとテストが進まなくなる
        # 一度描画するたびに2秒ポーズ
        if show_plot: plt.pause(2)
    
    ga.create_gif() # 保存した画像を繋ぎ合わせてGifを作成
    
    return True


# メイン関数の実行
if __name__ == "__main__":
    main()

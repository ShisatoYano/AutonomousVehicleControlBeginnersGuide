"""
Localization program by Dead Reckoning

Author: Shisato Yano
"""

import sys
import os
import matplotlib.pyplot as plt

# 他のディレクトリにあるモジュールを読み込むためのパス設定
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../motion_model")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../vehicle_drawing")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../common")
from four_wheels_vehicle import FourWheelsVehicle
from transformation import convert_speed_kmh_2_ms
from linear_motion.linear_motion_model import LinearMotionModel
from gif_animation import GifAnimation

# パラメータ定数
INTERVAL_SEC = 0.1
INTERVAL_MSEC = INTERVAL_SEC * 1000
TIME_LIMIT_SEC = 30
SPEED_KMH = 20
YAW_RATE_DS = 15

# 入力が含む誤差の標準偏差パラメータ
# これらの標準偏差を持った正規分布に従う誤差
SPEED_NOISE_STD = 1.0 # [m/sec]
YAW_RATE_NOISE_STD = 2.0 # [deg/sec]

# グラフの出力有無を切り替えるフラグ
show_plot = True


def main():
    print(__file__ + " + start!!")

    # 既に作成されている図があればクリア
    plt.clf()

    # 描画の設定
    ax = plt.subplot(1, 1, 1)
    ax.set_xlabel("X[m]")
    ax.set_ylabel("Y[m]")
    ax.set_aspect("equal")
    ax.set_title("Blue:Truth Red:Dead Reckoning")
    ax.grid(True)

    # 誤差の無い真値
    true_model = LinearMotionModel(interval_sec=INTERVAL_SEC) # モデル
    true_fwv = FourWheelsVehicle(ax, color='b') # 描画オブジェクト
    x_true, y_true, yaw_true, steer_true = 0.0, 0.0, 0.0, 0.0 # 計算される位置、方位角、ステア角
    x_true_all, y_true_all = [], [] # 計算されたx, y座標を記録する配列

    # 誤差を含んだ入力によるDead Reckoning
    dr_model = LinearMotionModel(speed_noise_std=SPEED_NOISE_STD, 
                                 yaw_rate_noise_std=YAW_RATE_NOISE_STD) # モデル
    dr_fwv = FourWheelsVehicle(ax, color='r') # 描画オブジェクト
    x_dr, y_dr, yaw_dr, steer_dr = 0.0, 0.0, 0.0, 0.0 # 計算される位置、方位角、ステア角
    x_dr_all, y_dr_all = [], [] # 計算されたx, y座標を記録する配列

    # Gif作成クラスのインスタンス生成
    save_name_path = os.path.dirname(os.path.abspath(__file__)) + "/../../gif/dead_reckoning.gif"
    ga = GifAnimation(save_name_path=save_name_path, duration_ms=INTERVAL_MSEC)

    elapsed_time_sec = 0.0 # 経過時間[s]
    speed_input, yaw_rate_input = 0.0, 0.0 # 入力する速度[m/s], 角速度[deg/s]

    # シミュレーション実行
    while TIME_LIMIT_SEC >= elapsed_time_sec:
        # 入力値の決定
        if elapsed_time_sec <= TIME_LIMIT_SEC/2:
            yaw_rate_input = YAW_RATE_DS
        else:
            yaw_rate_input = -YAW_RATE_DS
        speed_input = convert_speed_kmh_2_ms(SPEED_KMH)

        # 真値の計算
        x_true, y_true, yaw_true, steer_true = true_model.calculate_state(x_true, y_true, yaw_true, speed_input, yaw_rate_input)
        x_true_all.append(x_true), y_true_all.append(y_true)
        # 車両を描画
        true_fwv.draw(x_true, y_true, yaw_true, steer_true)

        # Dead Reckoningによる計算
        x_dr, y_dr, yaw_dr, steer_dr = dr_model.calculate_state(x_dr, y_dr, yaw_dr, speed_input, yaw_rate_input)
        x_dr_all.append(x_dr), y_dr_all.append(y_dr)
        # 車両を描画
        dr_fwv.draw(x_dr, y_dr, yaw_dr, steer_dr)

        # 車両の位置に合わせて描画範囲を更新
        ax.set_xlim([(x_true + x_dr)/2 - 20, (x_true + x_dr)/2 + 20])
        ax.set_ylim([(y_true + y_dr)/2 - 20, (y_true + y_dr)/2 + 20])

        # 時間を進める
        elapsed_time_sec += INTERVAL_SEC

        # ユニットテスト時はこのフラグをFlaseにする
        # グラフが表示されるとテストが進まなくなる
        if show_plot:
            ga.save_image() # 描画した図を画像として保存
            plt.pause(INTERVAL_SEC) # 一度描画するたびにインターバル時間分だけポーズ
    
    # 保存した画像を繋ぎ合わせてGifを作成
    ga.create_gif()

    # それぞれの走行軌跡を描画して比較
    plt.plot(x_true_all, y_true_all, ".b")
    plt.plot(x_dr_all, y_dr_all, ".r")
    plt.xlabel("X[m]")
    plt.ylabel("Y[m]")
    plt.axis("equal")
    plt.grid(True)
    
    if show_plot: plt.show()

    return True


# メイン関数
if __name__ == "__main__":
    main()

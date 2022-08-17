"""
Linear motion model comparison program

Author: Shisato Yano
"""

import sys
import os
import matplotlib.pyplot as plt

# 他のディレクトリにあるモジュールを読み込むためのパス設定
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../vehicle_drawing")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../common")
from four_wheels_vehicle import FourWheelsVehicle
from transformation import convert_speed_kmh_2_ms
from linear_motion_model import LinearMotionModel
from accurate_linear_motion_model import AccurateLinearMotionModel
from gif_animation import GifAnimation

# パラメータ定数
INTERVAL_SEC = 0.05
INTERVAL_MSEC = INTERVAL_SEC * 1000
TIME_LIMIT_SEC = 30
SPEED_KMH = 20
YAW_RATE_DS = 15

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
    ax.set_title("Blue:Normal Red:Accurate")
    ax.grid(True)

    # 通常の直線運動モデル
    lmm = LinearMotionModel(interval_sec=INTERVAL_SEC) # モデル
    lmm_fwv = FourWheelsVehicle(ax, color='b') # 描画オブジェクト
    x_lmm, y_lmm, yaw_lmm, steer_lmm = 0.0, 0.0, 0.0, 0.0 # 計算される位置、方位角、ステア角
    x_lmm_all, y_lmm_all = [], [] # 計算されたx, y座標を記録する配列

    # 高精度版の直線運動モデル
    almm = AccurateLinearMotionModel(interval_sec=INTERVAL_SEC) # モデル
    almm_fwv = FourWheelsVehicle(ax, color='r') # 描画オブジェクト
    x_almm, y_almm, yaw_almm, steer_almm = 0.0, 0.0, 0.0, 0.0 # 計算される位置、方位角、ステア角
    x_almm_all, y_almm_all = [], [] # 計算されたx, y座標を記録する配列

    # Gif作成クラスのインスタンス生成
    save_name_path = os.path.dirname(os.path.abspath(__file__)) + "/../../gif/compare_linear_model.gif"
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

        # 通常の運動モデルによる計算
        x_lmm, y_lmm, yaw_lmm, steer_lmm = lmm.calculate_state(x_lmm, y_lmm, yaw_lmm, speed_input, yaw_rate_input)
        x_lmm_all.append(x_lmm), y_lmm_all.append(y_lmm)
        # 車両を描画
        lmm_fwv.draw(x_lmm, y_lmm, yaw_lmm, steer_lmm)

        # 高精度版の運動モデルによる計算
        x_almm, y_almm, yaw_almm, steer_almm = almm.calculate_state(x_almm, y_almm, yaw_almm, speed_input, yaw_rate_input)
        x_almm_all.append(x_almm), y_almm_all.append(y_almm)
        # 車両を描画
        almm_fwv.draw(x_almm, y_almm, yaw_almm, steer_almm)

        # 車両の位置に合わせて描画範囲を更新
        ax.set_xlim([(x_lmm + x_almm)/2 - 20, (x_lmm + x_almm)/2 + 20])
        ax.set_ylim([(y_lmm + y_almm)/2 - 20, (y_lmm + y_almm)/2 + 20])

        # 時間を進める
        elapsed_time_sec += INTERVAL_SEC

        # ユニットテスト時はこのフラグをFlaseにする
        # グラフが表示されるとテストが進まなくなる
        if show_plot:
            ga.save_image() # 描画した図を画像として保存
            plt.pause(INTERVAL_SEC) # 一度描画するたびにインターバル時間分だけポーズ
    
    # 保存した画像を繋ぎ合わせてGifを作成
    ga.create_gif()

    # それぞれのモデルによる走行軌跡を描画して比較
    plt.plot(x_lmm_all, y_lmm_all, ".b")
    plt.plot(x_almm_all, y_almm_all, ".r")
    plt.xlabel("X[m]")
    plt.ylabel("Y[m]")
    plt.axis("equal")
    plt.grid(True)
    
    if show_plot: plt.show()

    return True


# メイン関数
if __name__ == "__main__":
    main()

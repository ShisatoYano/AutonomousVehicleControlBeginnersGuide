"""
Accurate linear motion model program

Author: Shisato Yano
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos, asin

# 他のディレクトリにあるモジュールを読み込むためのパス設定
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../vehicle_drawing")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../common")
from two_wheels_vehicle import TwoWheelsVehicle
from transformation import convert_speed_kmh_2_ms

# パラメータ定数
INTERVAL_SEC = 0.05
TIME_LIMIT_SEC = 30
SPEED_KMH = 20
YAW_RATE_DS = 15

# グラフの出力有無を切り替えるフラグ
show_plot = True


class AccurateLinearMotionModel:
    """
    直線運動モデルに従って車両の位置と方位を計算するクラス
    x, yの移動にて微小時間の真ん中の時刻の方位を使用する
    モデルによるオフセット誤差を少なくするための工夫
    """

    def __init__(self, front_length_m=6.35, rear_length_m=0.0, interval_sec=0.05):
        """
        コンストラクタ
        front_length_m: 車両位置から前方への長さ[m]
        rear_length_m: 車両位置から後方への長さ[m]
        interval_sec: 移動前後の刻み時間[sec]
        """

        # パラメータのセット
        self.wheel_base_m = front_length_m + rear_length_m # ホイールベース(前輪と後輪の間の距離)
        self.interval_sec = interval_sec

        # メンバ変数の初期化
        self.speed_ms = 0.0
        self.yaw_rate_ds = 0.0
        self.turning_radius_m = 0.0
        self.curvature = 0.0
    
    def calculate_state(self, x_m, y_m, yaw_deg, speed_ms, yaw_rate_ds):
        """
        速度[m/s]と角速度[deg/s]から車両の位置(x, y)と方位(yaw)を計算する関数
        x_m, y_m, yaw_deg: 1時刻前のx, y座標、方位yaw
        speed_ms: その時刻での速度[m/s]
        yaw_rate_ds: その時刻での角速度[deg/s]
        """

        # 入力
        self.speed_ms = speed_ms
        self.yaw_rate_ds = yaw_rate_ds

        # 直線運動モデルに従って位置と方位を計算
        theta = np.deg2rad(yaw_deg) + (np.deg2rad(yaw_rate_ds)*self.interval_sec/2)
        x_m_next = x_m + speed_ms * cos(theta) * self.interval_sec
        y_m_next = y_m + speed_ms * sin(theta) * self.interval_sec
        yaw_deg_next = yaw_deg + yaw_rate_ds * self.interval_sec
        
        # 速度と角速度からステアリング角度を計算
        # 速度が0になると0割りになるので注意
        steer_rad = asin(self.wheel_base_m * np.deg2rad(yaw_rate_ds)/speed_ms)
        
        return x_m_next, y_m_next, yaw_deg_next, np.rad2deg(steer_rad)


def main():
    print(__file__ + " + start!!")

    # 描画の設定
    ax = plt.subplot(1, 1, 1)
    ax.set_xlabel("X[m]")
    ax.set_ylabel("Y[m]")
    ax.set_aspect("equal")
    ax.grid(True)

    # 直線運動モデルのオブジェクト
    almm = AccurateLinearMotionModel(interval_sec=INTERVAL_SEC)

    # 2輪モデル車両の描画オブジェクト
    twv = TwoWheelsVehicle(ax)

    elapsed_time_sec = 0.0 # 経過時間[s]
    speed_input, yaw_rate_input = 0.0, 0.0 # 入力する速度[m/s], 角速度[deg/s]
    x_m, y_m, yaw_deg, steer_deg = 0.0, 0.0, 0.0, 0.0 # 計算される位置、方位角、ステア角

    # シミュレーション実行
    while TIME_LIMIT_SEC >= elapsed_time_sec:
        # 入力値の決定
        if elapsed_time_sec <= TIME_LIMIT_SEC/2:
            yaw_rate_input = YAW_RATE_DS
        else:
            yaw_rate_input = -YAW_RATE_DS
        speed_input = convert_speed_kmh_2_ms(SPEED_KMH)

        # 運動モデルによる計算
        x_m, y_m, yaw_deg, steer_deg = almm.calculate_state(x_m, y_m, yaw_deg, speed_input, yaw_rate_input)

        # 計算された位置と方位、ステアに従って車両を描画
        twv.draw(x_m, y_m, yaw_deg, steer_deg)

        # 車両の位置に合わせて描画範囲を更新
        ax.set_xlim([x_m - 15, x_m + 15])
        ax.set_ylim([y_m - 15, y_m + 15])

        # 時間を進める
        elapsed_time_sec += INTERVAL_SEC

        # ユニットテスト時はこのフラグをFlaseにする
        # グラフが表示されるとテストが進まなくなる
        # 一度描画するたびにインターバル時間分だけポーズ
        if show_plot: plt.pause(INTERVAL_SEC)
    
    return True


# メイン関数の実行
if __name__ == "__main__":
    main()

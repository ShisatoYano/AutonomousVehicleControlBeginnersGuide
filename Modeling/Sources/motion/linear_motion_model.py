"""
Linear motion model program

x_{t+1} = x_t + v_t * cos(yaw_t) * dt
y_{t+1} = y_t + v_t * sin(yaw_t) * dt
yaw_{t+1} = yaw_t * omega_t * dt

Author: Shisato Yano
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos, asin

# 他のディレクトリにあるモジュールを読み込むためのパス設定
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../vehicle")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../../Basis/Sources/transformation")
from four_wheels_vehicle import FourWheelsVehicle
from transformation import convert_speed_kmh_2_ms

# パラメータ定数
INTERVAL_SEC = 0.1
INTERVAL_MSEC = INTERVAL_SEC * 1000
TIME_LIMIT_SEC = 30
SPEED_KMH = 20
YAW_RATE_DS = 15

# グラフの出力有無を切り替えるフラグ
show_plot = True


class LinearMotionModel:
    """
    直線運動モデルに従って車両の位置と方位を計算するクラス
    """

    def __init__(self, front_length_m=2.0, rear_length_m=0.0, 
                 interval_sec=0.1, speed_noise_std=0.0,
                 yaw_rate_noise_std=0.0):
        """
        コンストラクタ
        front_length_m: 車両位置から前方への長さ[m]
        rear_length_m: 車両位置から後方への長さ[m]
        interval_sec: 移動前後の刻み時間[sec]
        speed_noise_std: 速度入力に含まれる誤差の標準偏差[m/sec]
        yaw_rate_noise_std: 角速度入力に含まれる誤差の標準偏差[deg/sec]
        """

        # パラメータのセット
        self.wheel_base_m = front_length_m + rear_length_m # ホイールベース(前輪と後輪の間の距離)
        self.interval_sec = interval_sec
        self.speed_noise_std = speed_noise_std
        self.yaw_rate_noise_std = yaw_rate_noise_std

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

        # 入力が含む誤差の共分散行列を定義
        # 正規分布に従った誤差を含むと仮定
        input_noise_covariance_matrix = \
            (np.diag([self.speed_noise_std, self.yaw_rate_noise_std]) ** 2) @ np.random.randn(2, 1) 

        # 誤差を含んだ入力
        self.speed_ms = speed_ms + input_noise_covariance_matrix[0, 0]
        self.yaw_rate_ds = yaw_rate_ds + input_noise_covariance_matrix[1, 0]

        # 直線運動モデルに従って位置と方位を計算
        x_m_next = x_m + self.speed_ms * cos(np.deg2rad(yaw_deg)) * self.interval_sec
        y_m_next = y_m + self.speed_ms * sin(np.deg2rad(yaw_deg)) * self.interval_sec
        yaw_deg_next = yaw_deg + self.yaw_rate_ds * self.interval_sec
        
        # 速度と角速度からステアリング角度を計算
        # 速度が0になると0割りになるので注意
        # asin -> ValueError: math domain error
        steer_rad = asin(self.wheel_base_m * np.deg2rad(self.yaw_rate_ds)/self.speed_ms)
        
        return x_m_next, y_m_next, yaw_deg_next, np.rad2deg(steer_rad)


def main():
    print(__file__ + " + start!!")

    # 既に作成されている図があればクリア
    plt.clf()

    # 描画の設定
    ax = plt.subplot(1, 1, 1)
    ax.set_xlabel("X[m]")
    ax.set_ylabel("Y[m]")
    ax.set_aspect("equal")
    ax.grid(True)

    # 直線運動モデルのオブジェクト
    lmm = LinearMotionModel(interval_sec=INTERVAL_SEC)

    # 四輪モデル車両の描画オブジェクト
    fwv = FourWheelsVehicle(ax)

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
        x_m, y_m, yaw_deg, steer_deg = lmm.calculate_state(x_m, y_m, yaw_deg, speed_input, yaw_rate_input)

        # 計算された位置と方位、ステアに従って車両を描画
        fwv.draw(x_m, y_m, yaw_deg, steer_deg)

        # 車両の位置に合わせて描画範囲を更新
        ax.set_xlim([x_m - 5, x_m + 5])
        ax.set_ylim([y_m - 5, y_m + 5])

        # 時間を進める
        elapsed_time_sec += INTERVAL_SEC

        # ユニットテスト時はこのフラグをFlaseにする
        # グラフが表示されるとテストが進まなくなる
        if show_plot: plt.pause(INTERVAL_SEC) # 一度描画するたびにインターバル時間分だけポーズ

    return True


# メイン関数の実行
if __name__ == "__main__":
    main()

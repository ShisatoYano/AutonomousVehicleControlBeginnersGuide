"""
GNSS positioning simulation program

Author: Shisato Yano
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt

# 他のディレクトリにあるモジュールを読み込むためのパス設定
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../Modeling/Sources/motion")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../Modeling/Sources/vehicle")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../common")
from four_wheels_vehicle import FourWheelsVehicle
from transformation import convert_speed_kmh_2_ms
from linear_motion_model import LinearMotionModel
from gif_animation import GifAnimation

# パラメータ定数
INTERVAL_SEC = 0.1
INTERVAL_MSEC = INTERVAL_SEC * 1000
TIME_LIMIT_SEC = 30
SPEED_KMH = 20
YAW_RATE_DS = 15

# GNSSで観測された位置座標が含む誤差の標準偏差パラメータ
# これらの標準偏差を持った正規分布に従う誤差
X_NOISE_STD = 0.5 # [m]
Y_NOISE_STD = 0.5 # [m]

# グラフの出力有無を切り替えるフラグ
show_plot = True


class GnssModel:
    """
    GNSSによる測位を模擬するクラス
    正規分布に従う誤差を含んだx, y座標を出力する
    """

    def __init__(self, axes, x_noise_std, y_noise_std, interval_sec, color):
        """
        コンストラクタ
        axes: 描画オブジェクト
        x_noise_std: x座標に含まれる誤差の標準偏差[m]
        y_noise_std: y座標に含まれる誤差の標準偏差[m]
        interval_sec: 測位される周期[sec]
        color: プロットする座標点の色
        """

        # パラメータのセット
        self.x_noise_std = x_noise_std
        self.y_noise_std = y_noise_std
        self.interval_sec = interval_sec
        self.color = color

        # 座標点を記録する配列
        self.x_points, self.y_points = [], []

        # 描画オブジェクトの初期化
        self.plot, = axes.plot(self.x_points, self.y_points, ".", color=self.color)

    def updated(self, elapsed_time_sec):
        """
        GNSS測位が得られたか判定する関数
        シミュレーション中の測位タイミングを制御するために使う
        """

        # interval_secで設定した値で経過時間が割り切れるならTrue
        # interval_sec周期で測位が得られることを模擬できる
        if round(elapsed_time_sec, 3) % self.interval_sec == 0.0: return True
        else: return False

    def observe(self, x_m, y_m):
        """
        測位された位置の座標点を計算する関数
        運動モデルで計算した位置に誤差パラメータを付加
        """

        # 測位された座標点が含む誤算の共分散行列を定義
        # 正規分布に従った誤差を含むと仮定
        noise_covariance_matrix = \
            (np.diag([self.x_noise_std, self.y_noise_std]) ** 2) @ np.random.randn(2, 1)
        
        # 誤差を含んだ座標点
        x_added_noise_m = x_m + noise_covariance_matrix[0, 0]
        y_added_noise_m = y_m + noise_covariance_matrix[1, 0]

        # 座標点を記録
        self.x_points.append(x_added_noise_m)
        self.y_points.append(y_added_noise_m)
    
    def draw(self):
        """
        測位された座標点を描画する関数
        時系列で記録されたものを全て描画
        """

        # 描画
        self.plot.set_data(self.x_points, self.y_points)
    
    def observed_points_x_m(self):
        """
        記録されたx座標点[m]を取得する関数
        """

        return self.x_points
    
    def observed_points_y_m(self):
        """
        記録されたy座標点[m]を取得する関数
        """

        return self.y_points


def main():
    print(__file__ + " + start!!")

    # 既に作成されている図があればクリア
    plt.clf()

    # 描画の設定
    ax = plt.subplot(1, 1, 1)
    ax.set_xlabel("X[m]")
    ax.set_ylabel("Y[m]")
    ax.set_aspect("equal")
    ax.set_title("Blue:Truth, Red:GNSS")
    ax.grid(True)

    # 誤差の無い真値
    true_model = LinearMotionModel(interval_sec=INTERVAL_SEC) # モデル
    true_fwv = FourWheelsVehicle(ax, color='b') # 描画オブジェクト
    x_true, y_true, yaw_true, steer_true = 0.0, 0.0, 0.0, 0.0 # 計算される位置、方位角、ステア角
    x_true_all, y_true_all = [], [] # 計算されたx, y座標を記録する配列

    # GNSSモデルのオブジェクト
    gnss = GnssModel(ax, X_NOISE_STD, Y_NOISE_STD, 0.5, 'r')

    # Gif作成クラスのインスタンス生成
    save_name_path = os.path.dirname(os.path.abspath(__file__)) + "/../../gif/gnss_model.gif"
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

        # GNSSによる測位
        # パラメータで指定した周期で更新
        if gnss.updated(elapsed_time_sec): 
            gnss.observe(x_true, y_true)
            gnss.draw()

        # 車両の位置に合わせて描画範囲を更新
        ax.set_xlim([x_true - 20, x_true + 20])
        ax.set_ylim([y_true - 20, y_true + 20])

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
    plt.plot(gnss.observed_points_x_m(), gnss.observed_points_y_m(), ".r")
    plt.xlabel("X[m]")
    plt.ylabel("Y[m]")
    plt.axis("equal")
    plt.grid(True)
    
    if show_plot: plt.show()

    return True


# メイン関数の実行
if __name__ == "__main__":
    main()

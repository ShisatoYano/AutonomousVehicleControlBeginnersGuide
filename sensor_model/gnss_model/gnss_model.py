"""
GNSS positioning simulation program

Author: Shisato Yano
"""

import matplotlib.pyplot as plt

# パラメータ定数
INTERVAL_SEC = 0.1
INTERVAL_MSEC = INTERVAL_SEC * 1000
TIME_LIMIT_SEC = 30

# グラフの出力有無を切り替えるフラグ
show_plot = True


class GnssModel:
    """
    GNSSによる測位を模擬するクラス
    正規分布に従う誤差を含んだx, y座標を出力する
    """

    def __init__(self, x_noise_std=0.5, y_noise_std=0.5, interval_sec=0.5):
        """
        コンストラクタ
        x_noise_std: x座標に含まれる誤差の標準偏差[m]
        y_noise_std: y座標に含まれる誤差の標準偏差[m]
        interval_sec: 測位される周期[sec]
        """

        # パラメータのセット
        self.x_noise_std = x_noise_std
        self.y_noise_std = y_noise_std
        self.interval_sec = interval_sec

    def updated(self, elapsed_time_sec):
        """
        GNSS測位が得られたか判定する関数
        シミュレーション中の測位タイミングを制御するために使う
        """

        # interval_secで設定した値で経過時間が割り切れるならTrue
        # interval_sec周期で測位が得られることを模擬できる
        if round(elapsed_time_sec, 3) % self.interval_sec == 0.0: return True
        else: return False


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

    # GNSSモデルのオブジェクト
    gnss = GnssModel()

    elapsed_time_sec = 0.0 # 経過時間[s]

    # シミュレーション実行
    while TIME_LIMIT_SEC >= elapsed_time_sec:
        if gnss.updated(elapsed_time_sec):
            print(elapsed_time_sec, " GNSS updated!!")

        # 時間を進める
        elapsed_time_sec += INTERVAL_SEC

    return True


# メイン関数の実行
if __name__ == "__main__":
    main()

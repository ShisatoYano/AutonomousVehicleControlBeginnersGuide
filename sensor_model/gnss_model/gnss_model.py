"""
GNSS positioning simulation program

Author: Shisato Yano
"""


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

"""
GNSS positioning simulation program

Author: Shisato Yano
"""


class GnssModel:
    """
    GNSSによる測位を模擬するクラス
    """

    def __init__(self, x_noise_std=0.5, y_noise_std=0.5, interval_sec=0.5):
        """
        コンストラクタ
        
        """

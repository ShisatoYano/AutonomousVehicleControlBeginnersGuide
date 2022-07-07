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

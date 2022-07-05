"""
Linear motion model program

Author: Shisato Yano
"""

import sys
from os.path import dirname, abspath

# 他のディレクトリにあるモジュールを読み込むためのパス設定
# parent_dir = dirname(dirname(dirname(abspath(__file__))))
# if parent_dir not in sys.path: sys.path.append(parent_dir)
# from vehicle_drawing import two_wheels_vehicle

from ...vehicle_drawing.two_wheels_vehicle import TwoWheelsVehicle


class LinearMotionModel:
    """
    直線運動モデルに従って車両の位置と姿勢を計算するクラス
    """

    def __init__(self, front_length_m=6.35, rear_length_m=0.0, interval_sec=0.05):
        """
        コンストラクタ
        front_length_m: 車両位置から前方への長さ[m]
        rear_length_m: 車両位置から後方への長さ[m]
        """

        # パラメータのセット
        self.wheel_base_m = front_length_m + rear_length_m # ホイールベース(前輪と後輪の間の距離)
        self.interval_sec = interval_sec # 移動前後の刻み時間[sec]
    

# メイン処理
if __name__ == "__main__":
    print(__file__ + " + start!!")



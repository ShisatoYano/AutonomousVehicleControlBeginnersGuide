# motion_model
車両の運動モデルに基づいて位置と姿勢を計算するプログラムをまとめています。  

## 目次
* [運動モデルとは](#運動モデルとは)
* [直線運動モデル](#直線運動モデル)
* [四輪モデル](#四輪モデル)
* [二輪モデル](#二輪モデル)

## 運動モデルとは
制御対象である車両の動きを理論的に定式化したものです。車両自身の移動速度や姿勢角速度、ステアリング舵角に応じて、次の時刻における位置と姿勢がどうなるかを予測したり、仮想環境での制御シミュレーションを実施したりするのに利用されます。  
こういった運動モデルを扱うためには、車両の運動学(ビークルダイナミクス)の知識が必要になるので、ここではその基礎を学び、その計算式をPythonで実装出来るようになることを目指します。 

## 直線運動モデル
### 理論式
まず一番シンプルな運動モデルとして、直線運動モデルというものがあります。これは、ある時刻$t$から次の時刻$t+1$の間を移動する際の車両の動きを直線で近似したものです。そして、このモデルによって時刻$t+1$における車両の位置と方位を計算する式は次のようになります。  
$$
x_{t+1} = x_t + v_t cos\theta_t dt \\
y_{t+1} = y_t + v_t sin\theta_t dt \\
\theta_{t+1} = \theta_t + \omega_t dt
$$  
この式における$v_t$は時刻$t$の時の車両の速度、$\omega_t$は同じく時刻$t$の時の車両の方位角速度です。速度に時間をかけることで移動量が求まるので、上の式に含まれるこれらの部分は、時刻$t$から時刻$t+1$の間の車両の位置座標$x, y$と方位角$\theta$の変化量ということになります。  
$$
v_t cos\theta_t dt \\
v_t sin\theta_t dt \\
\omega_t dt
$$
その変化量を時刻$t$の時の位置座標$x_t, y_t$と方位角$\theta_t$に足し合わせることで、次の時刻$t+1$の時の位置座標$x_{t+1}, y_{t+1}$と方位角$\theta_{t+1}$が求まるという訳です。  

### Pythonでの実装
こちらのファイルに、直線運動モデルのPythonコードを実装しています。  
/motion_model/linear_motion_model/linear_motion_model.py  
```python
"""
Linear motion model program

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


class LinearMotionModel:
    """
    直線運動モデルに従って車両の位置と方位を計算するクラス
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
        x = x_m + speed_ms * cos(np.deg2rad(yaw_deg)) * self.interval_sec
        y = y_m + speed_ms * sin(np.deg2rad(yaw_deg)) * self.interval_sec
        yaw = yaw_deg + yaw_rate_ds * self.interval_sec
        
        # 速度と角速度からステアリング角度を計算
        # 速度が0になると0割りになるので注意
        steer_rad = asin(self.wheel_base_m * np.deg2rad(self.yaw_rate_ds)/self.speed_ms)
        
        return x, y, yaw, np.rad2deg(steer_rad)
    

# メイン処理
# このファイルを実行すると、運動モデルに従って
# 車両の位置と方位を計算するシミュレーションが
# 実行される
if __name__ == "__main__":
    print(__file__ + " + start!!")

    # 描画の設定
    ax = plt.subplot(1, 1, 1)
    ax.set_xlabel("X[m]")
    ax.set_ylabel("Y[m]")
    ax.set_aspect("equal")
    ax.grid(True)

    # 直線運動モデルのオブジェクト
    lmm = LinearMotionModel(interval_sec=INTERVAL_SEC)

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
        x_m, y_m, yaw_deg, steer_deg = lmm.calculate_state(x_m, y_m, yaw_deg, speed_input, yaw_rate_input)

        # 計算された位置と方位、ステアに従って車両を描画
        twv.draw(x_m, y_m, yaw_deg, steer_deg)

        # 車両の位置に合わせて描画範囲を更新
        ax.set_xlim([x_m - 15, x_m + 15])
        ax.set_ylim([y_m - 15, y_m + 15])

        # 時間を進める
        elapsed_time_sec += INTERVAL_SEC

        # アニメーション更新を遅くしたい場合はここで一瞬ポーズさせる
        plt.pause(INTERVAL_SEC)

```


## 円運動モデル

## ステアリング型ロボットの運動モデル
![](/images/steering_robot.png)  

## 四輪モデル
車両の運動モデルとしてまず取り上げられるのは、こちらのような四輪モデルです。  
車両のダイナミクスを考える際は、このような2次元平面の力学、運動学を考えるのが一般的です。  
![](/images/four_wheels_model.png)  

この図の中で定義されている各変数は下記のようになります。  
* $L_f$, $L_r$: 車両の重心位置から前後輪それぞれまでの距離  
* $L$($L_f$+$L_r$): ホイールベース(前後輪間の距離)  
* $d$: トレッド(左右のタイヤ間の距離)
* $V_{fL}$, $V_{fR}$, $V_{rL}$, $V_{rR}$: 前後左右の各タイヤの速度ベクトル  
* $V_b$: 車両の重心位置の速度ベクトル
* $\delta_{fL}$, $\delta_{fR}$: 左右前輪のそれぞれのステアリング舵角
* $\beta_{fL}$, $\beta_{fR}$, $\beta_{rL}$, $\beta_{rR}$: 前後左右の各タイヤのすべり角
* $\beta$: 車両全体のすべり角
* $\dot{\phi}$: 車両方位の角速度
* $CF$: 旋回時に車両にかかる遠心力

このように、前後左右の4つのタイヤにはそれぞれ別々の速度ベクトルやすべり角が生じると考えるのが、四輪モデルの特徴です。

## 二輪モデル
四輪モデルは車両のより細かい動きを表現できる反面、それを実データから解析してモデルとして構築するのが難しいというデメリットもあります。そこで、前後それぞれの左右のタイヤの速度ベクトルやすべり角は同一になると考えることで、モデルをシンプルにするという方法があります。それがこちらの二輪モデルです。  
![](/images/two_wheels_model.png)  

上記の考え方により、左右別々にあったタイヤは一つだとみなすことになり、その形状からBicycle Modelとも呼ばれます。これに伴い、モデルの中で扱う変数の数もかなり少なくなります。  
* $L_f$, $L_r$: 車両の重心位置から前後輪それぞれまでの距離  
* $L$($L_f$+$L_r$): ホイールベース(前後輪間の距離)  
* $V_{r}$, $V_{r}$: 前後のタイヤの速度ベクトル  
* $V_b$: 車両の重心位置の速度ベクトル
* $\delta$: 前輪のそれぞれのステアリング舵角
* $\beta_{f}$, $\beta_{r}$: 前後各タイヤのすべり角
* $\beta$: 車両全体のすべり角
* $\dot{\phi}$: 車両方位の角速度
* $CF$: 旋回時に車両にかかる遠心力


# common
各サンプルプログラムで共通に使える関数をまとめておくところです。  

## 目次
* [transformation](#transformation)
    * [座標変換とは](#座標変換とは)
    * [座標変換の分解](#座標変換の分解)
    * [回転](#回転)
    * [平行移動](#平行移動)
    * [Pythonでの実装](#pythonでの実装)

## transformation
座標系や単位、値の範囲などを変換する関数をまとめたものです。  

### 座標変換とは  
自動運転システムでは、車両自体の位置座標や向き、RadarやLiDARのような外界センサが検知した物体の位置座標や向きなどのデータを扱いますが、これら(特にセンサから得られるデータ)は通常基準となる座標系が異なるため、同じ座標系を基準とした値となるようにしないといけません。このための変換処理を座標変換といいます。  

例えば車両自体は、決して動くことのない座標系を基準とした位置座標と向きの情報を持ちます。この座標系をワールド座標系(X-Y)と呼び、最終的には全ての位置座標と向きの情報をワールド座標系基準となるように変換することが多いです。  
![](/images/multiple_coordinates.png)  

しかしながら、車両に搭載されているセンサが検知した障害物の位置座標と向きについては、車両の位置と向きを基準にした車両座標系(U-V)として扱いたい場合もあります。こういうときは、ワールド座標系から車両座標系へ、あるいはその逆の変換が必要になります。  

### 座標変換の分解
このようにAとBという異なる2つの座標系がある場合を考えてみましょう。  
![](/images/coordinate_a_b.png)  

ここで座標系Aを基準にしていた状態から座標系Bを基準にした状態に変換したいとき、  
変換処理は次の2つのステップに分けて考えることができます。  
まず1つ目のステップは、それぞれの座標系の向きを合わせる回転です。  
![](/images/rotation.png)  

そして最後に、それぞれの座標系の原点が一致させるためにずらす平行移動です。  
![](/images/translation.png)  

### 回転
回転という変換を具体的に言うと、このように座標系A(X-Y)を基準とした点Pの位置座標を、  
座標系B(U-V)を基準としたものにすることです。  
![](/images/rotate_a_b.png)  

このとき、X-Y軸の単位ベクトルを$\boldsymbol{x}$, $\boldsymbol{y}$, U-V軸の単位ベクトルを$\boldsymbol{u}$, $\boldsymbol{v}$、両者の成す角を$\theta$とすると、  
$$
\boldsymbol{u} = cos\theta\boldsymbol{x} + sin\theta\boldsymbol{y} \\
\boldsymbol{v} = -sin\theta\boldsymbol{x} + cos\theta\boldsymbol{y}
$$
となります。  

また、点Pを示すベクトル$p$は、  
$$
\boldsymbol{p} = p_x\boldsymbol{x}+p_y\boldsymbol{y} = p_u\boldsymbol{u}+p_v\boldsymbol{v}
$$  
と、それぞれの座標軸の単位ベクトル成分に分解できます。  

つまり、座標系Aを基準にしたベクトル$p$は、  
$$
^A\boldsymbol{p} = (p_x, p_y)^T
$$  
座標系Bを基準にしたベクトル$p$は、  
$$
^B\boldsymbol{p} = (p_u, p_v)^T
$$
となります。  

以上の式を結合すると、  
$$
p_x\boldsymbol{x} + p_y\boldsymbol{y} = p_u(cos\theta\boldsymbol{x} + sin\theta\boldsymbol{y}) + p_v(-sin\theta\boldsymbol{x} + cos\theta\boldsymbol{y}) \\
= (p_ucos\theta - p_vsin\theta)\boldsymbol{x} + (p_usin\theta + p_vcos\theta)\boldsymbol{y}
$$  
となり、ここから  
$$
p_x = p_u cos\theta - p_v sin\theta \\
p_y = p_u sin\theta + p_v cos\theta
$$  
という2式が導けます。これをまとめると、  
$$
\begin{pmatrix}
p_x \\ p_y
\end{pmatrix} = 
\begin{pmatrix}
cos\theta & -sin\theta \\ sin\theta & cos\theta
\end{pmatrix}
\begin{pmatrix}
p_u \\ p_v
\end{pmatrix}
$$  
となります。つまり、座標系Aを基準としたベクトル$^A\boldsymbol{p}$は、両座標系の成す角$\theta$の$cos$成分と$sin$成分を要素に持つ2X2の行列と、座標系Bを基準としたベクトル$^B\boldsymbol{p}$を掛け合わせたものと考えることができます。この計算式をプログラムに実装することで、異なる座標系間のある点の位置座標と向きの回転を実行することができます。  

また、この式は、  
$$
^A\boldsymbol{p} = ^A\boldsymbol{R}_B(\theta)^B\boldsymbol{p}
$$
$$
^A\boldsymbol{R}_B =
\begin{pmatrix}
cos\theta & -sin\theta \\ sin\theta & cos\theta
\end{pmatrix}
$$
と簡潔に書くことができ、$^A\boldsymbol{R}_B$を、座標系Bから座標系Aに変換するための回転行列と言います。  

### 平行移動
平行移動は、単純に座標をオフセットさせるだけの処理です。  
例えばこのように異なる座標系A, Bがあり、それぞれから見た点Pの位置座標を$^A\boldsymbol{p}$, $^B\boldsymbol{p}$とします。  
![](/images/translation_a_b.png)  

自動運転システムでは、座標系B基準で見ていた座標Pを座標系A基準で見たらどうなるか、あるいはその逆は、といった演算が度々必要となります。特に多いのは、座標系Aをワールド座標系、座標系Bを車両座標系としたときに、車両座標系上での位置座標Pをワールド座標系上での位置座標に変換するこちらの式です。  
$$
^A\boldsymbol{p} = ^B\boldsymbol{p} + \boldsymbol{q}
$$  
この式にある$\boldsymbol{q}$は、座標系Aから見た座標系Bの原点位置であり、$^B\boldsymbol{p}$にその分を足し合わせることで、座標系Aから見た$^A\boldsymbol{p}$になるという訳です。  

### Pythonでの実装
ここまで説明した回転 + 平行移動の座標変換を行うPythonプログラムは下記のように実装できます。  
```python
def rotate_translate_2d(points, x_m, y_m, angle_rad):
    """
    2次元点群を同次変換する関数
    angle_degだけ回転させて、x_m, y_mだけ平行移動
    points: 2 x 点数の2次元配列, np.array([[x1, x2, x3], [y1, y2, y3]])など
    x_m: X軸方向の平行移動量
    y_m: Y軸方向の平行移動量
    angle_rad: 回転角度[rad]
    """
    
    # 回転角度の単位をradにして、sin, cos成分を計算
    angle_cos = cos(angle_rad)
    angle_sin = sin(angle_rad)

    # 回転行列を定義
    rotation_matrix = np.array([[angle_cos, -angle_sin], [angle_sin, angle_cos]])
    
    # 入力点群を回転
    rotated_points = rotation_matrix.dot(points)
    
    # 回転させた点群を平行移動
    transformed_points = rotated_points + np.ones(points.shape) * (np.array([[x_m], [y_m]]))
    
    return transformed_points # 回転 + 平行移動させた点群
```

このプログラムは、ここまでに説明してきた図の座標系B上で見た点の2次元位置座標$^B\boldsymbol{p}=(p_u, p_v)^T$を入力pointsとし、それを座標系Aとの成す角angle_rad分だけ回転、そして座標系Aから見た座標系Bの原点位置x_m, y_m分だけ平行移動させることで座標系A上で見た位置座標に変換する計算を関数化したものです。  

まず、変換の対象となる入力位置座標pointsは、下記のような2次元配列の形式で与えるようにします。  
```python
np.array([[x1, x2, x3], [y1, y2, y3]])
```
これは2 X 入力する座標点の数の2次元配列を定義したものであり、数式としてはこのようになります。  
$$
\begin{pmatrix}
p_{x1} & p_{x2} & p_{x3} \\ p_{y1} & p_{y2} & p_{y3}
\end{pmatrix}
$$  
こうすることで、複数の点に対してまとめて座標変換できるようにしています。  

続いて、回転行列$^A\boldsymbol{R}_B$はこちらのように実装できます。  
```python
angle_cos = cos(angle_rad)
angle_sin = sin(angle_rad)

rotation_matrix = np.array([[angle_cos, -angle_sin], [angle_sin, angle_cos]])
```
そして、実装したrotation_matrixとpointsを掛け合わせることで回転による変換を実現するのですが、Pythonではそれをこのようなコードで書くことができます。  
```python
rotated_points = rotation_matrix.dot(points)
```

最後に、回転による変換が施された点であるrotated_pointsに対して平行移動による変換を施して、一連の座標変換処理を完了させます。その計算はこのようなコードで実装できます。  
```python
transformed_points = rotated_points + np.ones(points.shape) * (np.array([[x_m], [y_m]]))
```
ここでポイントになるのは、式の右辺の後半にある  
```python
np.ones(points.shape) * (np.array([[x_m], [y_m]]))
```
です。rotated_pointsは2 X 点の数となる2次元配列なので、それに平行移動分を足し合わせるためには配列のサイズを一致させる必要があります。そこで、上記のようなコードにすることで平行移動分のベクトルをrotated_pointsと同じサイズにすることができます。  

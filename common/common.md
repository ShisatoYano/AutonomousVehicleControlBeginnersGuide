# common
各サンプルプログラムで共通に使える関数をまとめておくところです。  

## 目次
* [transformation](#transformation)
    * [座標変換とは](#座標変換とは)
    * [座標変換の分解](#座標変換の分解)
    * [回転](#回転)
    * [平行移動](#平行移動)

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

このとき、X-Y軸の単位ベクトルを、$\boldsymbol{x}$, $\boldsymbol{y}$、U-V軸の単位ベクトルを$\boldsymbol{u}$, $\boldsymbol{v}$、両者の成す角を$\theta$とすると、  
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
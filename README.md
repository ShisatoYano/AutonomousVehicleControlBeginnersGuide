# AutonomousDrivingSamplePrograms
[![Linux_CI](https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms/actions/workflows/Linux_CI.yml/badge.svg)](https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms/actions/workflows/Linux_CI.yml) [![Windows_CI](https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms/actions/workflows/Windows_CI.yml/badge.svg)](https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms/actions/workflows/Windows_CI.yml) [![MacOS_CI](https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms/actions/workflows/MacOS_CI.yml/badge.svg)](https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms/actions/workflows/MacOS_CI.yml) [![CodeFactor](https://www.codefactor.io/repository/github/shisatoyano/autonomousdrivingsampleprograms/badge)](https://www.codefactor.io/repository/github/shisatoyano/autonomousdrivingsampleprograms)  
自動運転のアルゴリズムについて学ぶためのサンプルプログラム集。  

## 目次
* [このリポジトリについて](#このリポジトリについて)
* [動作条件](#動作条件)
* [ディレクトリ構成](#ディレクトリ構成)

## このリポジトリについて
ロボットの自律移動や車両の自動運転に必要な各種要素技術の一般的なアルゴリズムについて学ぶためのサンプルプログラムをまとめています。内容の理解や実装がしやすいように、Pythonで書くものがメインになります。また、各プログラムで実装された技術について解説したドキュメントも作成しているので、合わせて参照ください。  

## 動作条件
各サンプルプログラムを動かすためには下記条件を満たしている必要があります。  
* Python 3.10.x以上
* numpy
* matplotlib
* pytest
* pytest-cov

## 使い方
1. このリポジトリを自身のローカルにクローンします。  
```bash
$ git clone https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms.git
```
2. 必要なPython用ライブラリをインストールします。  
```bash
$ pip install -r requirements.txt
```
3. 下記のように各ディレクトリにあるPythonスクリプトを実行します。  
```bash
$ python motion_model/linear_motion/linear_motion_model.py
```
4. もし気に入っていただけたら、スターを付けていただけると嬉しいです。

## ディレクトリ構成
* [common](/common/common.md): 各プログラムで共通で使える関数
* [images](/images/): 各ドキュメント用の画像ファイル
* [motion_model](/motion_model/motion_model.md): 運動モデルで車両の位置と姿勢を計算するプログラム
* [test](/test/test.md): 各プログラムのユニットテストコード
* [vehicle_drawing](/vehicle_drawing/vehicle_drawing.md): 車両の絵を描画するプログラム

## 作者
* [Shisato Yano](https://github.com/ShisatoYano)

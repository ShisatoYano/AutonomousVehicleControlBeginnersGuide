# AutonomousDrivingSamplePrograms
[![Linux_CI](https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms/actions/workflows/Linux_CI.yml/badge.svg)](https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms/actions/workflows/Linux_CI.yml) [![Windows_CI](https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms/actions/workflows/Windows_CI.yml/badge.svg)](https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms/actions/workflows/Windows_CI.yml) [![MacOS_CI](https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms/actions/workflows/MacOS_CI.yml/badge.svg)](https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms/actions/workflows/MacOS_CI.yml) [![CodeFactor](https://www.codefactor.io/repository/github/shisatoyano/autonomousdrivingsampleprograms/badge)](https://www.codefactor.io/repository/github/shisatoyano/autonomousdrivingsampleprograms)
自動運転のアルゴリズムについて学ぶためのサンプルプログラム集。  

## 目次
* [このリポジトリについて](#このリポジトリについて)
* [動作条件](#動作条件)
* [ディレクトリ構成](#ディレクトリ構成)

## このリポジトリについて
ロボットの自律移動や車両の自動運転に必要な各種要素技術の一般的なアルゴリズムについて学ぶためのサンプルプログラムをまとめています。内容の理解や実装がしやすいように、Pythonで書くものがメインになります。また、各プログラムで実装された技術について解説したドキュメントも作成していくので、合わせて参照ください。  

## 動作条件
* Python 3.10.x以上
* numpy
* matplotlib
* pytest
* pytest-cov

もしこれらライブラリが不足している場合は、下記コマンドを実行してインストールしてください。  
```bash
$ pip install -r requirements.txt
```

## ディレクトリ構成
* [common](/common/common.md): 各プログラムで共通で使える関数
* [images](/images/): 各ドキュメント用の画像ファイル
* [motion_model](/motion_model/motion_model.md): 運動モデルで車両の位置と姿勢を計算するプログラム
* [test](/test/test.md): 各プログラムのユニットテストコード
* [vehicle_drawing](/vehicle_drawing/vehicle_drawing.md): 車両の絵を描画するプログラム

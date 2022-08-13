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
1. このリポジトリを自分のPCのローカルにクローンします。  
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

## 本プロジェクトへの貢献
バグの報告や改善要望などありましたらご自由にIssuesに登録ください。  
コード追加や変更などのプルリクエストも大歓迎です。プルリクエストを出していただく際は、[こちらの手順](/docs/contributing_guide.md)に従ってください。また、コード変更時は[こちらのルール](/docs/test_guide.md)に従ってテストを実行し、既存コードに不具合が起きていないことを確認してからプルリクエストを出すようにしてください。  

## お問い合わせ
ご質問等ありましたら、こちらのメールアドレスまでご連絡ください。  
shisatoyano@gmail.com  

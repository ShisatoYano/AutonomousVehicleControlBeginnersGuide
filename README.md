# AutonomousVehicleControlBeginnersGuide
[![Linux_CI](https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms/actions/workflows/Linux_CI.yml/badge.svg)](https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms/actions/workflows/Linux_CI.yml) [![Windows_CI](https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms/actions/workflows/Windows_CI.yml/badge.svg)](https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms/actions/workflows/Windows_CI.yml) [![MacOS_CI](https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms/actions/workflows/MacOS_CI.yml/badge.svg)](https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms/actions/workflows/MacOS_CI.yml) [![CodeFactor](https://www.codefactor.io/repository/github/shisatoyano/autonomousvehiclecontrolbeginnersguide/badge)](https://www.codefactor.io/repository/github/shisatoyano/autonomousvehiclecontrolbeginnersguide)  

## Contents
* [Preface](#preface)
* [Composition](#composition)
    * [Basis](#basis)
    * [Modeling](#modeling)
    * [Localization](#localization)
    * [PathPlanning](#pathplanning)
    * [VehicleControl](#vehiclecontrol)
    * [Navigation](#navigation)
* [How to run sample code](#how-to-run-sample-code)
* [プログラムの構成](#プログラムの構成)
    * [common](#common)
    * [motion_model](#motion_model)
    * [vehicle_drawing](#vehicle_drawing)
* [本プロジェクトへの貢献](#本プロジェクトへの貢献)
* [ライセンス](#ライセンス)
* [お問い合わせ](#お問い合わせ)

## Preface
This repository is beginner's guide to learn basic way of thinking and representative algorithms for Autonomous vehicle control. Explanation documents and sample codes about Motion model, Localization, Path Planning/Following and Vehicle Control are included in this repository. All of them are themed typical 4 wheels drive vehicle. I hope you can understand the above algorithms practically by reading documents and implementing codes.  

## Composition
This repository is composed of multiple directories as follow. Each directory has explanation documents, source codes and image files about component technologies for autonomous vehicle control system.  

### Basis
Studying the basic knowledge of Probability and Statistics.  

### Modeling
Formulating vehicle's motion and observation by sensor and implement their simulation.  

### Localization
Understanding and implementing algorithm of Self-Localization and SLAM.  

### PathPlanning
Understanding and implementing algorithm of Global/Local Path Planning.  

### VehicleControl
Understanding and implementing algorithm of Vehicle Control and Path Tracking.  

### Navigation
Design scenario in case of navigating autonomous vehicle from start to goal and implement algorithm to achieve the navigation.  

Each directory have the following 3 sub directories.  

* Documents
* Sources
* Images

"Documents" directory has explanation documents. In each document, an theory of algorithm is explained in detail while mixing formulas and sample codes. You can understand the algorithm by reading explanations and implementing codes practically.  
"Sources" directory has source files of sample codes introduced in documents. These codes are written in Python. Usually, almost all of the codes for autonomous vehicle system are written in C/C++ but it is very difficult for beginner to understand and implement. So, all of the codes in this repository are written in Python because you can understand and implement a code more easily and more roughly than C/C++.  
"Images" direcotry has image or gif files shown in documents. These files are created as output by each sample codes and you can confirm how an algorithm behave by seeing them.  

## How to run sample code
Each sample codes are implemented by Python and some libraries.  
The following Python version and libraries are required.  

* Newer than Python 3.10.x
* numpy
* matplotlib
* pytest
* pytest-cov

You can run a code by following this procedure.  

1. Clone this repository.  
```bash
$ git clone https://github.com/ShisatoYano/AutonomousVehicleControlBeginnersGuide.git
```
2. Install required libraries.  
```bash
$ pip install -r requirements.txt
```
3. Execute python script in each directory.  
```bash
$ python Localization/Sources/kalman_filter/linear_kalman_filter_1d.py
```
4. Add star to this repository if you like it. 

## プログラムの構成
本リポジトリのサンプルプログラムは、以下のような構成でジャンルや目的別のディレクトリに分けられて配置されています。  
### common
プログラム全体で共通で使えるモジュールをまとめています。  
詳細については、コードに書かれたコメントや[こちらのドキュメント](/common/common.md)を参照ください。  

### motion_model
車両の動きを理論的に計算するための運動モデルを実装したモジュールをまとめています。  
各種モデルを実装したクラスのコードは全て単体で実行できるようになっており、実行すると以下のように運動モデルに従った車両の動きがアニメーションで描画されます。  
![](/gif/linear_motion_model.gif)  
また、このディレクトリにある運動モデルクラスをimportすることで、位置計測や車両制御などのアルゴリズムのシミュレーションにも使うことができます。運動モデルの理論の詳細については[こちらのドキュメント](/motion_model/motion_model.md)を参照ください。  

### vehicle_drawing
各種シミュレーションを実行したときの車両の動きをアニメーションとして可視化するモジュールをまとめています。主に二輪モデルと四輪モデルの車両の二種類を描画でき、それぞれはボディやタイヤなどのパーツモジュールの集合体として表現されています。例えば、モジュールの一つであるfour_wheels_vehicle.pyを実行すると、このように四輪モデルの車両の描画例を見ることができます。  
![](/gif/four_wheels_vehicle.gif)  
詳細については、[こちらのドキュメント](/vehicle_drawing/vehicle_drawing.md)を参照ください。  

## 本プロジェクトへの貢献
バグの報告や改善要望などありましたらご自由に[Issues](https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms/issues)に登録ください。  
コード追加や変更などのプルリクエストも大歓迎です。プルリクエストを出していただく際は、[こちらの手順](/docs/contributing_guide.md)に従ってください。また、コード変更時は[こちらのルール](/docs/test_guide.md)に従ってテストを実行し、既存コードに不具合が起きていないことを確認してからプルリクエストを出すようにしてください。  


## お問い合わせ
ご質問等ありましたら、こちらのメールアドレスまでご連絡ください。  
shisatoyano@gmail.com  

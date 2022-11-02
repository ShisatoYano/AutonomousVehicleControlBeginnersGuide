# Testing standards
This document is about standards of Unit test.  
Each sample programs are modularized and can be executed independently. Some of programs are implemented by importing multiple modules. When you create a pull request, please confirm all of programs can be executed correctly by Unite test in advance.  

## テストコードの作り方
テストコードは全てtestディレクトリに置くようにしてください。また、各テストコードのファイル名はtest_テスト対象ファイル名.pyにしてください。例えば、テスト対象ファイルがhoge.pyである場合は、テストコードファイル名をtest_hoge.pyとしてtestディレクトリ以下に配置します。  

## フレームワーク
各テストコードは、Pythonスクリプトをテストするためのフレームワークである[pytest](https://github.com/pytest-dev/pytest)を使用しています。pytestの使い方については、[こちらの記事](https://note.com/npaka/n/n84de488ba011)の説明が分かりやすいので参照ください。  

## テストの実行方法
全てのテストコードをまとめて実行する際は、本リポジトリのルートディレクトリにある下記のスクリプトを実行してください。実行環境がLinuxの場合は、  
```bash
$ ./run_test_suites.sh
```
Windowsの場合は、  
```bash
.\run_test_suites.bat
```
をそれぞれ実行することで、testディレクトリ以下にあるtest_から始まるファイルをテストコードとして自動探索してユニットテストしてくれます。また、全てではなく特定のテストコードのみを実行したい場合は、  
```bash
$ pytest test/test_hoge.py
```
というようにして実行します。そして、テストの結果がこのようにErrorが出ず全てPassすればOKです。  
![](/images/unit_test.png)  

## GitHub Actionsによる自動テスト
リモートリポジトリのmainブランチにコードがpushされた際は、[GitHub Actions](https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms/actions)により自動で全てのユニットテストが実行されるようになっています。こちらのような3パターンのOSの仮想実行環境がGitHubサーバ上で構築され、上記のテストスクリプトが実行されます。  
![](/images/test_workflows_os.png)  
サーバ上での環境構築やテスト実行のログは、このようにGitHub Actionsのページから確認できます。  
![](/images/actions_test_example.png)  


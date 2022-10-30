# 貢献していただく際のルール
バグ報告や改善要望、プルリクエストなど、誰でも自由に本プロジェクトへ貢献していただくことができます。その際は、下記に定めるにルールに従っていただけると幸いです。  

## バグの報告
もし、本プロジェクトのサンプルプログラムのバグやドキュメントの不備を見つけた際は、遠慮なく[Issues](https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms/issues)に登録してください。その際は、下記の情報を含めて登録していただけると助かります。  

* バグの概要
* 正しくはどうあるべきか
* Pythonのバージョン、各ライブラリのバージョン

## 改善要望
既存のサンプルプログラムがもっと～だと良い、新しく～のサンプルコードを入れてほしい、といった改善要望についても、遠慮なく[Issues](https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms/issues)に登録してください。  

## プルリクエスト
既存のサンプルプログラムのコード修正や新規コードの追加を提案していただける場合は、是非[こちら](https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms/pulls)へプルリクエストを作成してください。その際は、下記の手順に従っていただけると助かります。  

1. もしGitHubアカウントをお持ちでなければ作成してください。
2. 本プロジェクトの[リポジトリ](https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms)を自分のGitHubアカウントにForkして、自分が所有するリポジトリとして扱える様にしてください。  
3. Forkしたリポジトリをローカルにクローンしてください。
```bash
git clone https://github.com/<YOUR GITHUB USERNAME>/AutonomousDrivingSamplePrograms.git
```
4. 本リポジトリをupstreamとしてgit configに登録してください。  
```bash
git remote add upstream https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms.git
```
5. [README](../README.md)に記載している本リポジトリの使い方に従って必要なライブラリをインストールし、既存のサンプルプログラムが動作することを確認ください。  
6. 作業用のブランチを作成し、コードの追加や変更を行ってください。決してmainブランチ上では作業を行わないでください。    
```bash
git branch -b my_feature origin/main
```
7. 追加、修正作業が完了したら、それらの変更をcommit & pushする前に、下記のスクリプトでユニットテストを実行してください。このとき、OSがWindowsである場合は、run_test_suites.batを代わりに実行してください。テストの実行に関する詳細なルールについては、[こちらのドキュメント](/docs/test_guide.md)を参照してください。  
```bash
$ ./run_test_suites.sh
```
8. ユニットテストでExceptionなどが出ずに全てのテストがPassすることを確認できたら、変更をリモートにcommin & pushしましょう。  
9. リモートにpushが出来たら、Forkした自分のリポジトリのWebページ上に"Compare & pull request"というボタンが出ているはずなので、それをクリックします。すると、Pull Requestに添付するコメントを入力する画面になるので、コメントを入力して、"Create pull request"を押してください。  
10. 9にて提出されたプルリクエストを私の方でレビューしますので、修正などが必要になった場合は対応をお願い致します。  
11. 最終的に全て問題無ければマージを指示させていただくので、そしたらmainブランチへのマージをお願い致します。  

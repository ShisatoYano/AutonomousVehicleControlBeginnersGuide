# CLAUDE.md

このプロジェクト固有の方針。共通の進め方は `~/.claude/CLAUDE.md` を参照。

## プロジェクトの目的

自動運転アルゴリズムの学習用サンプル集(将来的に技術書として書籍化予定)。
最優先はアルゴリズムの理解しやすさであり、実用性・パフォーマンスではない。
実装を検討する際は「教材として分かりやすいか」を常に優先する。

## 参照すべきドキュメント

- `README.md`: プロジェクト概要、対応アルゴリズム一覧、セットアップ手順
- `HOWTOCONTRIBUTE.md`: 新規アルゴリズム追加のフロー(Issue提案→実装→テスト→PR)
- `doc/DESIGN_DOCUMENT.md`: 各プログラムの設計ドキュメント(整備中)

## ディレクトリ構成

- `src/components/`: 再利用可能なモジュール(vehicle, control, state, course, mapping, search, sensors, detection, obstacle, plan, array, visualization, common)
- `src/simulations/`: 各アルゴリズムの実行スクリプト。カテゴリ別(localization, mapping, path_planning, path_tracking, perception, course)
- `test/`: `src/simulations` 配下のシミュレーションに対応する `test_*.py`(ほぼ1:1対応)
- `doc/`: 設計ドキュメント

## コーディング規約(既存コードから読み取れるパターン)

- Pythonのみ。標準的なパッケージ構造は取らず、各スクリプトが `sys.path.append(...)` で `src/components/` 配下のモジュールを実行時に読み込む(既存ファイルのパターンを踏襲する)
- 各ファイル冒頭に `"""ファイル名 \n\n Author: <名前>"""` 形式のdocstringを置く
- クラス・コンストラクタにはdocstringで各引数の意味・単位を明記する(例: `f_len_m`, `max_accel_mps2` のように変数名に単位を付ける命名規則)
- シミュレーションスクリプトは `show_plot` フラグを持ち、`main()` 関数にロジックをまとめる(テスト時に `show_plot = False` でアニメーション表示を止められるようにするため)
- 新しいモジュールディレクトリ(`src/components/*` または `src/simulations/*` 配下)を追加したら、ホスト側で `python generate_pyrightconfig.py` を実行して `pyrightconfig.json` と `.devcontainer/devcontainer.json` を再生成する

## テスト

- 実行: `. run_test_suites.sh`(内部は `pytest -l --durations=0`)
- 新しいシミュレーションを追加したら対応する `test/test_<simulation_name>.py` を追加する。中身は `show_plot = False` を設定してから `main()` を呼ぶだけのシンプルなスモークテストが基本パターン(アニメーション表示なしで例外なく実行できることを確認する)

## 開発フロー

新規アルゴリズム追加は `HOWTOCONTRIBUTE.md` のStep 1〜4(Issue提案→実装→テスト→PR)に従う。

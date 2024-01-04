# micro-ROS Arduino examples for Pi:Co Classic3 with TMC5072 motor driver

[![Compile Sketches](https://github.com/rt-net/adit_pico_micro_ros_arduino_examples/actions/workflows/compile-sketches.yaml/badge.svg)](https://github.com/rt-net/adit_pico_micro_ros_arduino_examples/actions/workflows/compile-sketches.yaml)
[![Lint](https://github.com/rt-net/adit_pico_micro_ros_arduino_examples/actions/workflows/lint.yaml/badge.svg)](https://github.com/rt-net/adit_pico_micro_ros_arduino_examples/actions/workflows/lint.yaml)

**TMC5072モータドライバIC**を搭載したPi:Co Classic3用のmicro-ROS Arduinoサンプルスケッチ集です。

[オプションキット No.1 [ESP32-S3マイコンボード]](https://www.rt-shop.jp/index.php?main_page=product_info&products_id=4131)
を搭載したPi:Co Classic3を、
**Arduino**および**micro-ROS**で動かせます。

## サンプルスケッチについて

### STEP5, STEP8

- micro-ROSを使用しないサンプルスケッチです

### STEP10

- micro-ROSを使用するサンプルスケッチです

## スケッチファイルの自動整形について

ソースコードのレイアウトを整えるため、各スケッチファイルにはArduino IDEの自動整形を適用しています。
自動整形のルールは[.clang-format](.clang-format) ファイルを参照してください。

## License

(C) 2024 RT Corporation

各ファイルはライセンスがファイル中に明記されている場合、そのライセンスに従います。特に明記されていない場合は、Apache License, Version 2.0に基づき公開されています。  
ライセンスの全文は[LICENSE](./LICENSE)または[https://www.apache.org/licenses/LICENSE-2.0](https://www.apache.org/licenses/LICENSE-2.0)から確認できます。

※このソフトウェアは基本的にオープンソースソフトウェアとして「AS IS」（現状有姿のまま）で提供しています。本ソフトウェアに関する無償サポートはありません。  
バグの修正や誤字脱字の修正に関するリクエストは常に受け付けていますが、それ以外の機能追加等のリクエストについては社内のガイドラインを優先します。

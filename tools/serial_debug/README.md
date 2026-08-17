# STM32 シリアルデバッグツール

NUCLEO-F767ZI のシリアルログを JSON Lines（JSONL）で保存し、AI が扱いやすい要約 JSON を出力する Python CLI です。STM32CubeProgrammer CLI を利用した ST-LINK 経由の書込み／リセットも行えます。

## セットアップ

Python 3.9 以降と STM32CubeProgrammer を用意してください。

```sh
python3 -m pip install -r tools/serial_debug/requirements.txt
python3 tools/serial_debug/capture.py --json list-ports
```

Linux でポートを開けない場合は、ユーザーを `dialout` グループへ追加し、再ログインします。

```sh
sudo usermod -aG dialout "$USER"
```

## Codex 向けの一括診断

通常は `diagnose` だけを呼び出してください。シリアルポートを開いて収集待機状態にしてから、`--reset` 指定時のみ ST-LINK 経由でリセットし、指定時間のログを取得します。生ログは標準出力へ出さず JSONL にのみ保存し、終了時に異常箇所の前後だけを含む短い JSON を 1 件返します。

```sh
python3 tools/serial_debug/capture.py diagnose \
  --port /dev/ttyACM0 --baudrate 115200 --duration 20 --reset \
  --output tools/serial_debug/logs/f767-diagnose.jsonl
```

終了コードは、異常なしが 0、警告・エラー・YAMLルールに一致する異常、またはリセット失敗が 1、実行不能が 2 です。デフォルトでは JSONL を最大 2,000 イベントまたは 2 MiB に制限し、上限を越えた件数は要約の `log.omitted` で確認できます。検出対象は [events.yaml](events.yaml) で変更でき、HardFault、MemManage、BusFault、UsageFault、ウォッチドッグ、スタックオーバーフローなどを初期設定しています。

要約は最大5件の異常について、そのイベントと前後3行だけを保持します。必要なら `--context-before`、`--context-after`、`--max-anomalies`、`--max-events`、`--max-bytes` を調整できます。

## 個別のログ収集と要約

ポート名とボーレートは環境に合わせて指定します。NUCLEO の ST-LINK 仮想 COM ポートは多くの場合 `/dev/ttyACM0` です。

```sh
python3 tools/serial_debug/capture.py --json capture \
  --port /dev/ttyACM0 --baudrate 115200 --duration 30 --echo \
  --output tools/serial_debug/logs/f767.jsonl

python3 tools/serial_debug/capture.py --json summarize \
  tools/serial_debug/logs/f767.jsonl
```

`capture` は各受信行を `stm32.serial_log.v1` の JSONL イベントとして追記します。UTF-8 として復元できないバイトは `message` で置換し、元のバイト列は `raw_hex` に残します。`summarize` はレベル別件数、直近イベント、`ERROR`／`FATAL`／`PANIC` のイベントを `stm32.serial_summary.v1` JSON として出力し、エラーがある場合は終了コード 1 を返します。

## 書込みとリセット

次の操作は実機を変更します。対象ボードとファームウェアを確認してから、AI または利用者が明示的に実行してください。

```sh
# Debug ビルド後、ELF を検証付きで書込み、リセット
cmake --preset Debug
cmake --build --preset Debug
python3 tools/serial_debug/capture.py --json flash \
  build/Debug/26catch_f7 --reset

# ST-LINK 経由でリセットのみ
python3 tools/serial_debug/capture.py --json reset --method stlink
```

STM32CubeProgrammer CLI が PATH にない場合は、`--programmer /path/to/STM32_Programmer_CLI` を指定します。`.bin` を書き込む場合のデフォルト先頭アドレスは `0x08000000` で、必要に応じて `--address` を変更できます。

`reset --method dtr-rts --port /dev/ttyACM0` は USB-シリアル変換器の DTR/RTS をパルスします。NUCLEO の標準 ST-LINK VCP がこの配線でリセットされるとは限らないため、通常は `stlink` を使用してください。

## AI からの利用指針

`diagnose` の標準出力は常に1件の JSON 要約です。個別操作で `--json` を指定すると、各操作結果を JSON として判定できます。`capture --echo --json` は受信イベントを JSONL として逐次出力するため、Codex 向けの一括診断には使用しないでください。`flash` と `reset` は、明示的なコマンド実行時のみ実機に作用します。ログ収集・要約・ポート列挙は実機の状態を変更しません。

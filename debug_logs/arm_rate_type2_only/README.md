# Robstride DOB: Type2のみの500Hz経路（2026-09-09）

## 実装

通常モードとDisable中の位置・速度・iqfのType17読み出しは100Hzを維持。
Robstride Mode4のEnable中は、500Hzの目標電流書き込みに返るType2の位置・速度・トルクを使用し、周期的なType17読み出しは行わない。
ROSのcurrentはID1（EL05）ではトルク/0.94、ID2（RS02）ではトルク/1.22で求めるArms相当の参考値であり、実測iqfではない。
位置・速度・トルクは同一フレームのスナップショットとして保持する。速度は標準FBの速度にLPFを適用し、粗い位置の差分からは求めない。
実測iqfが必要な場合の明示的なType17読み出し経路は残している。サービスの確認経路は変更しない。
Robstride Mode5 MPCは未実装であり、今回の実機確認はMode4のみ。標準FBの共通APIは将来利用可能。

## ビルド・実機確認

Debug/Releaseビルド、速度推定ホストテスト、diffチェック成功。書き込み・Verify済み。
ELF SHA256: `c24d92f1e23b7fd9e9a69adb4bb3d8127471ace2652cc72d4c000cf53c12e365`

2台同時Mode4、速度目標±0.2deg/s・0.5Hzの低振幅試験を実施。
10秒試験のROSログ: `../arm_step_20260909_190527/`
このディレクトリの`rate.csv`では、両軸が稼働した完全な1秒窓11個（保持時間を含む）すべてで以下を確認。

- 各軸の目標送信受付500回/s、Type2受信500回/s。
- Type17の位置・速度・電流受信は各0回/s。
- 送信リングあふれ、優先キュー満杯、送信エラー、CANエラーはいずれも0。
- 制御ループ500回/s、最大ループ間隔2ms。
- Disable後はType17位置・速度・電流が各100回/sに復帰。

続けて30秒試験も正常終了: `../arm_step_20260909_190637/`。
ただし30秒試験後のRAM取得は遅れ、120秒リングから稼働区間が消えていたため、30秒全体の500Hzカウンター連続性は未確認。
500Hzの根拠は上記10秒試験の保存済みカウンターであり、ROSのFB周期は従来の100Hzのまま。
これは通信・制御周期の確認であり、速度追従性能や電流値の精度を保証する試験ではない。
試験終了後は全モーターDisable。RoboMasterの起動校正は無効のまま。競合していた手動Launchも停止したまま。

## CAN速度とメーカー資料

CAN1は48MHz/(Prescaler6 × 8TQ)=1Mbps。
メーカー記載の選択肢1M/500K/250K/125Kの上限であり、ボーレート引き上げではなく通信量削減で対応。
新経路は2軸合計で概ね2000フレーム/s（各軸500書き込み+500応答）。

- [RS02メーカー資料](https://github.com/RobStride/Product_Information/blob/main/Product%20Literature/RS02/RS02User%20Manual260713.pdf)
- [EL05メーカー資料](https://github.com/RobStride/Product_Information/blob/main/Product%20Literature/EL05/EL05User%20Manual260713.pdf)

Type18書き込みへのType2応答には位置・速度・トルク・温度が含まれる。トルク定数の単位はNm/Arms。

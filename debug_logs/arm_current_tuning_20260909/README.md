# Robstride内部電流ループ 初期比較（2026-09-09）

追記：18:21以降のID1追加比較は [id1_refine/README.md](id1_refine/README.md) を参照。
追加候補は採用せず下記設定へ復元済み。長い繰り返しでID1の不足が再現しており、調整完了ではない。
下記の最終位置は初回比較終了時点の履歴。現在の停止位置は追加比較側に記載。

## 現在書き込み済み設定

ID1（肘・EL05）、ID2（根本・RS02）とも cur_kp=0.17、cur_ki=0.012、cur_filt_gain=0.10。
変更前は0.05 / 0.05 / 0.06。Core/Src/can_devices.cで設定。
F7起動時に3項目をType18で書き、アドレス別Type17新着応答で読み戻す。
一致しなければEnableを拒否する。EEPROM保存はせず、F7起動ごとに設定する。
kp017_boot.jsonlに両IDのreadback成功記録あり。
Debug/Releaseビルド成功、VSCode同梱CubeProgrammerで書き込み・verify成功。
最終Debug ELF SHA256: ee0525bb8a3bee9d496678dff536c1dafa92ef9104e770e135f0d61df26ad970

## 資料

メーカー公開資料をmanuals/にPDFとテキストで保存。

- https://github.com/RobStride/Product_Information/blob/main/Product%20Literature/RS02/RS02User%20Manual260713.pdf
- https://github.com/RobStride/Product_Information/blob/main/Product%20Literature/EL05/EL05User%20Manual260713.pdf
- https://github.com/RobStride/Product_Information/blob/main/Product%20Literature/RS05/RS05User%20Manual260713.pdf

Type17/18公開パラメーター表の既定値はKp=.17、Ki=.012、filter=.1。
別のStudioパラメーター表の参考値とは区別した。0x701A iqfはフィルタ済み電流。
Studio表には0x3020 iq原電流もあるが、今回は読み出しておらず公開Type17での対応は未検証。

## 試験条件

対象軸は電流モード（ROS mode2、モーターrun_mode3）で±0.2Aを各250ms、ゼロ400ms、3往復。
他軸は現在位置をPP保持。ROS指令は約100Hz。電流FBはiqf。
各試験後両RobstrideをDisable。RoboMasterは起動校正・Enableとも無効化を維持。
F7の±145度保護、試験開始から10度の変位監視を維持。5A試験はしていない。
各ログmetadata.jsonに設定ソース・ローカルELFハッシュ・終了状態を保存。
各250msパルスの最後80msの電流中央値で比較。ホスト受信時刻であり、内側FOCの帯域測定ではない。

|タグ|Kp|Ki|filter|ID1ログ末尾|ID2ログ末尾|
|---|---:|---:|---:|---|---|
|baseline|.05|.05|.06|181112|181526|
|filter010|.05|.05|.10|181636|181646|
|kp010|.10|.05|.10|181742|181752|
|ki0012|.10|.012|.10|181845|181855|
|kp017|.17|.012|.10|181955|182005|

ログは ../arm_step_20260909_上記末尾/feedback.csv。全10試験complete。
current_comparison.pngとcomparison.jsonに比較を保存。
再生成: PYTHONNOUSERSITE=1 python3 tools/analyze_arm_current.py（リポジトリ直下）。

## 結果と未完了事項

根本ID2：変更前の定常電流絶対値は約0.108～0.137A、最終は0.197～0.205A。
肘ID1：変更前0.134～0.152A、最終0.170～0.202A。後半パルスで不足が残り、原因未特定。
これらは小電流・ほぼ静止状態の観測結果。温度・姿勢・順序は独立に統制していない。
約100Hzのフィルタ済みFBだけで実電流の高周波応答・安定余裕を保証できない。
ピーク/RMS換算による不足と断定せず、指令を機械的にsqrt(2)倍する等の補正は入れていない。
現在値は暫定採用。次は再現性と振幅依存、必要なら原電流読出しを確認してからDOB再調整。
非対称な機構モデルは追加していない。Robstrideの位置MPCはまだ未実装。
最終FB: ID1=-116.9282度、ID2=105.8705度、両軸state=0（Disable）。

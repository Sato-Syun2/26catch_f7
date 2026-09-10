# 5台統合ファームウェア（2026-09-10）

## 追加変更：新しい位置指令でRobstride再Enable（書き込み済み）

ユーザー指示でReleaseを書き込み・ベリファイ・リセット成功。
SHA256 `2612d05ada24d435afccd8ba8aeeae5af15bbcbf544b3c0aa098b41fb1827ffb`。
書き込み時はmicro-ROS Agent未起動。Agentは新たに起動しておらず、変更後の実機移動は未検証。

ユーザー指定により、RobstrideのPOS/Mode5では「初回だけ」を撤廃。
Disable・指令タイムアウト・Enable失敗後も、新しい有効な位置指令で自動Enableを試みる。
有効中はEnableを重複送信しない。有限値・位置目標範囲・指令世代・FB・既存保護チェックは維持。
速度／電流モードの停止後自動復帰は今回の変更対象外。
位置指令が連送されている間はDisableサービス後も次の新着指令で再始動するため、
停止を維持するときは指令送信元も止める必要がある。
Debug/Release、全ホスト試験（自動Enable条件の回帰試験追加）、LwIP／スタック確認成功。
GUIへの操作は行っていない。buildのELFは更新済みなので、
下記の実機測定時ELFとは別バージョン。

## 最新実機結果：Releaseで連続目標100Hz・4軸MPC完走（15:13 JST）

- 現在書き込み済みはRelease、SHA256 `e63e66a522dc3769d106b152902b5ea0ca3b88f95f8015449d5e5fe2d7248d10`。SWD書き込み・ベリファイ成功。下記の範囲復帰／3秒停滞変更も含む。
- `../integrated_mpc_live_20260910_151154/`: Robstride2軸は2deg幅、C610 ID4/C620は10mm幅の0.4Hz目標を100Hzで連続送信。60秒＋終了時整定1.38秒を完走。4軸とも定常中state1/code1、クラッシュ・再起動・途中停止なし。C610 ID1はDisableのまま。最終全5台state0/code1、C620 fault=0。
- 同一Release ELFから解決したRAMログ58件: Robstride **495.01～500.50Hz、最大間隔5ms**。CAN ring overrun・priority full・TX error・CAN errorすべて0、Type2受信も各軸最低495.01Hz。ROS FB100.0007Hz、定常最大受信間隔17.52ms。最小空きスタックRobstride7196B/RoboMaster3268B。
- Debugは修正後も別の同時往復60秒試験で426.57～434.13Hz、最大間隔8ms（`../integrated_mpc_live_20260910_150911/`）。同じソースのReleaseで持続的低下は解消したが、厳密な2ms締切保証ではない。CPU処理時間・スケジューリングの影響が疑われ、CAN飽和を示すエラーはない。Debug/Releaseは完全に同じ軌道・起動状態ではないため、単一箇所への原因断定はしない。
- 最初のDebug試験 `../integrated_mpc_live_20260910_150639/` は約27秒でC620 fault=2により中断、全台停止成功。受信鮮度判定でHAL_GetTickより後にISR更新時刻を読むと符号なし差がアンダーフローし得たため、**受信時刻→現在時刻**の読み順に修正。10msしきい値は変更なし。修正後のDebug60秒＋Release連続60秒では再発なし。初回の直接原因はログ粒度不足で断定しない。
- M3508の停滞原点は `../integrated_mpc_live_20260910_150905/` で実機確認。`c620_origin=stall_unverified`、Mode5保持に復帰。シリアル `freshness_fix_serial.jsonl` に15:09:11の完了記録。Release起動ではスイッチ完了。故意に機構範囲外へ移動させる実機試験は行っていない（範囲外からの復帰はホスト試験済み）。
- 試験スクリプトも修正: 初回topic自動Enableと明示Enableを重ねない、Ctrl-C後もDisable完了までROS contextを維持、停止処理を別phaseで記録、書き込みELFを指定してハッシュ記録。Release段階目標試験 `../integrated_mpc_live_20260910_151107/` はユーザーの連続目標への切替指示で中断。旧Ctrl-C処理でcontextが先に閉じたためsummary未生成だが、その後別のROS照会で全5台Disableを確認してから連続試験へ移った。
- ユーザー方針: ロボコンの試合継続を重視。瞬間的な遅延と持続的な通信断を区別し、誤判定や過剰なラッチを減らす。今回、保護を一括解除したり通信断しきい値を根拠なく延長したりはしていない。
- Debug/Releaseビルド、既存ホスト全試験、LwIP配置・スタック予算チェック成功。未コミット・未Push。peripheral launchは継続。

## 最新変更：位置範囲からの復帰とM3508停滞原点（未書き込み）

C610 ID4/C620の位置モードは範囲外目標をSetTargetで拒否し、以前の有効目標を保持する。実位置が範囲外という理由だけでは異常をラッチせず、範囲内への復帰を許可する。ID4の位置モードでは実測速度による予測端停止ラッチを適用しない。MPCの速度包絡、通信・非有限値保護は維持。Robstrideの保護およびID4の速度モードの予測停止は変更していない。

M3508/C620は位置変動0.15mm以内・速度絶対値2mm/s未満が3秒継続すると電流ゼロに移り、さらに500ms停止確認後に原点登録する。移動中は継続し、60秒上限のフォールバックも維持。完了理由は `stall_unverified`（停滞）または `timeout_unverified`（時間切れ）。途中の引っ掛かりを真の原点と区別できない。スイッチ押下での即時完了は維持。

ホスト試験で上下限外からの内向きMPC出力・範囲外目標拒否・3秒停滞と停止確認を確認。Debug/Releaseビルド、全ホスト試験、LwIP配置・スタック予算チェック成功。この変更は実機未書き込み。

直前の周期修正版（C610 ID1換算31×π÷36×2、タスク期限一致時の余分な待機を解消、Robstride MPCの連続目標更新でも50Hz演算を維持）は書き込み済み。`../integrated_mpc_live_20260910_145700/` の4軸連続目標試験は約61.54秒で正常終了、全台Disable確認。ただし修正後の実制御周期のRAMログ解析は未完了。

## 最新の実機結果：4軸同時MPC（14:44～14:49 JST）

**タイムアウト原点登録版を書き込み済み。4台同時MPCの往復60秒＋連続目標60秒でクラッシュ・再起動・通信停止なし。最後は全5台Disable確認。ただしDebug版はRobstride制御約315～332Hzで、目標500Hz未達。** 以下の未書き込み表記はこの試験前の履歴。

- 書き込み・ベリファイ成功。ELF SHA256 `46150e6031d9afc8cc2f14e328f3f407193d690a48b4ebc04c10ada5e557d8f6`。
- 起動校正 `../integrated_mpc_live_20260910_144425/`: C620はスイッチ未検出で60秒後、ゼロ電流・停止確認を経て原点登録・保持。`c620_origin=timeout_unverified`、state5。Robstrideは指令前Disable。
- 初回指令Enable `../integrated_mpc_live_20260910_144537/`: Robstride2台ともEnableサービスなしで各2deg移動成功。その後Disable直後のType2→Type17切替を試験側が異常判定し中断。停止サービスはすべて成功。以後、Disable確認後150ms以内のstate0のみ切替猶予を追加（有効軸・200ms以上のFB途絶は除外しない）。
- 単独＋同時往復 `../integrated_mpc_live_20260910_144633/`: Robstride各2deg、C610 ID4/C620各10mmの単独移動成功。4軸同時に5秒ごと12区間・60秒往復し到達判定成功、停止要求もすべて成功。
- 連続目標 `../integrated_mpc_live_20260910_144819/`: 0.4Hzの位置目標を100Hzで4軸へ更新。Robstrideは2deg幅、ID4/C620は10mm幅。60秒の連続動作と終了時到達判定成功。全4軸の実機ctrl_type下位バイトが5であることをHot Plug読み取りで確認（`continuous_modes_stack.txt`）。
- 連続試験の定常FB約100.03Hz、最大受信間隔18.67ms。定常中、対象4軸は全サンプルstate1/code1。
- RAM周期ログ59件: Robstride制御315.37～332.00Hz、最大ループ間隔9ms。CAN ring overrun・priority full・TX error・CAN errorはいずれも0。**クラッシュ解消の確認と500Hz周期達成は別。速度モデルの2ms前提に対する実周期不足は今後の最適化・Release実機検証課題。**
- 最小空きスタックRobstride7040B/8192B、RoboMaster3264B/4096B。旧1KBでは実測使用量1152B相当が収まらない。RAMダンプ `final_rates_stack.txt`、解析 `../integrated_mpc_live_20260910_144819/analysis.json`。
- 最後のC620 fault=0、全5台state0/code1。指定peripheral launchは起動継続。C610 ID1はMPC未調整のため同時移動に含めずDisable。
- 小移動・低速の負荷確認であり、最大速度・全可動域・長時間運転の保証ではない。機構原点はスイッチ未確認。

### 追加変更：C620タイムアウト原点登録（未書き込み）

ユーザー指定により、スイッチ未反応でも起動時の明示操作を不要にした。校正は従来の5A・60秒上限。60秒経過後は電流指令をゼロにし、速度絶対値2mm/s未満かつ位置変動0.15mm以内が500ms継続したら、現在位置を原点登録し既存のMode5保持へ移る。停止待ちが2秒を超えても安定しなければ異常停止。通信途絶・非有限値・校正速度異常・過大移動・明示Disableなどは校正成功に変換しない。

この方式は真の機構原点と途中の引っ掛かりを区別できない。`c620_origin` は `timeout_unverified` を返し、シリアルにもスイッチ未確認の原点登録として記録する。ユーザーは誤った引っ掛かり時にはリセットする運用を指定しているが、同じ引っ掛かりが続けばリセット後も同じ原点になる。実機の端・範囲確認を代替する検証ではない。

ホスト試験で停止待ち中のゼロ電流、原点登録が一度だけ、保持復帰、通信／移動継続／非有限値／取消の異常分離を確認。Debug/Release・LwIP・スタック予算チェックPASS。この追加変更と押下済み起動変更はまだ実機へ未書き込み。

### 追加変更：起動時C620スイッチON（未書き込み）

C620のPA5が起動時点でHighなら、従来の1秒待ちを省略し、新鮮なFBを取得した最初の制御周期で原点確定・校正完了へ進む。同じ周期の校正電流フレームはゼロ、Mode5位置保持へ復帰する。PA5がLowの場合は従来の1秒待ちと通常校正を維持。FB未受信・古いFBでは原点確定しない。押下済み起動／FB待ち／再校正されないことをホスト試験で確認し、Debug・ReleaseビルドとLwIP配置検証成功。この追加変更はまだ実機に書き込んでいない。

> 最新状態（14:32 JST）: スタック拡張・Robstride初回指令Enable版を書き込み・ベリファイ済み。5台の通信、Robstride起動時Disable、C610両軸の校正後保持を確認。C620は校正途中で停滞し、試験終了時に全台Disable。能動移動・初回指令Enable・同時MPC負荷は未再試験。

## 再接続・修正版書き込み（14:32 JST）

- 指定された `catch26_core/launch/peripheral.launch.py` をROS_DOMAIN_ID=30で起動。micro-ROS Agent、Kondo、Feetechを起動し、serial bridgeとFeetechのLifecycle ACTIVEを確認。launchは起動したまま。
- 有線LANの既存プロファイル `catch2026-f7-arm-usblan` を有効化。
- launch既定動作でFeetechの角度範囲設定が適用された（servo1/2/5の更新をログで確認）。servo5は初期位置が設定範囲外との警告あり。こちらからサーボ移動指令は送っていない。
- 書き込み直前のFBは4台構成だった。既存4台をDisableしてから、ST-LINK `066DFF363445503043032152` に5台統合Debug ELFを書き込み・ベリファイ・リセット成功。
- ELF SHA256: `dffe609d5bea8f2e4e3cf6a290ad76d98b890469c97429c0a01ea258088ad772`。
- 起動ログ `first_command_stackfix_boot.jsonl`、観測ログ `../integrated_smoke_20260910_143246/`。
- Robstride2台はstate0で待機。C610 ID4・ID1は原点付近でstate1（位置保持）。C620はstate2（校正中）、約−2.06mmで約4.81A、8秒の観測中に校正完了せず。スクリプトのresult=completeは観測処理完了であり、全台校正成功ではない。
- 観測終了時のDisableは全5台応答あり、最終FBすべてstate0/code1。C620のスイッチ照会はPA5=0（未作動）。追加の校正・移動は行っていない。
- Hot Plugの非停止RAM読み取りで最小空きスタック: Robstride7668B、RoboMaster3392B。ただしRobstrideのMPC移動前の値であり、動作中の余裕は未検証。

## 最新オフライン修正・通信断調査

- Robstride起動時自動Enableを撤去。Mode5・Disableで待機し、有効な初回ROS指令を受けた軸だけ自動Enableする。非有限値／MPC範囲外目標は自動Enableしない。Disable・モード変更で世代が変わった指令でもEnableしない。
- 初回Enable失敗・明示Disable・タイムアウト後は通常の明示Enableを使用。ROSセッション再構築では既存の初回指令待機初期化に従う。
- RoboMasterの校正成功後保持は維持。C620は校正成功時に承認済み端180.145263671875mm／上限175.145263671875mmを復元する。
- 前回実機のHot Plug読み取り（書き込み・リセットなし）で、RobstrideTaskの1KBスタックの底まで書き込み済み、制御ループ最終tick=0x7e6fに対しシステムtick=0x38019を確認。Pingは応答していた。
- Debugコンパイラの`.su`では、制御タスクからCAN送信に至る既知のネストだけで旧構成約1056B、修正版でも1040B。旧1024B割り当てではFPU例外／RTOSコンテキスト以前に不足する。スタック不足は確認できるが、これだけで通信断の全原因・解消を実機確認したとは扱わない。
- RobstrideTask: 256→2048ワード（1→8KB）。RobomasTask: 256→1024ワード（1→4KB、複数制御器の余裕確保）。IOCの静的タスク定義も同期。
- `robstride_stack_free_min_bytes` / `robomas_stack_free_min_bytes` に毎秒最小空きスタック量[B]を記録。次回実機でMPC移動・Enable/Disable中の値を確認する。
- `bash tools/check_control_stacks.sh` で既知のDebug呼び出し経路＋512Bの余裕とIOC構成を検証。これは全経路の上限保証ではない。
- Debug/Releaseビルド、ホスト回帰、LwIP配置検証成功。Debug RAM445232B（90.58%）、Flash283528B、Release RAM445176B（90.57%）、Flash201404B。
- 再書き込み・初回指令自動Enable・各軸移動・同時負荷の実機検証は、接続復帰後に行う。

## 自動保持版の試験（14:14～14:15 JST）

- Robstrideはモード設定・リミット確認後、制御タスクで新鮮な現在位置を取得してMode5 Enable。RoboMasterは校正完了後の原点を目標にEnable。C620は確認済み範囲を校正成功時に復元する。
- 起動保持中のみROS指令タイムアウトを除外し、初回topic指令以後は通常監視へ戻す。Disable・モード変更で起動保持を解除し、自動再Enableしない。
- Debug/Release・ホスト回帰・LwIP配置検証成功。自動保持のID4/C620ホストケースを追加。Debug RAM434992B、Flash283520B、Release RAM434936B、Flash201316B。
- 書き込み・ベリファイ成功。起動ログ `startup_hold_boot.jsonl`。
- 起動観測 `../integrated_smoke_20260910_141415/`: Robstride根元59.49deg、肘109.26deg付近でstate1、ID4原点state1、C620原点0.0526mm・state1を確認。ID1はstate2（校正中）・約2Aが継続したためDisable。観測終了時は全軸Disable確認。
- ID2小移動 `../integrated_smoke_20260910_141446/`: 初期59.3905degから−5deg目標。途中でFB途絶。全軸Disableを試みたが5サービスともタイムアウト。最後の受信ID2は57.2470deg・state1。最終スナップショットは古い受信値であり停止の証拠ではない。
- Pingは応答あり。ROS FB再取得・別プロセスからのDisableも応答確認できず。原因未確定。ユーザーにモーター電源遮断を依頼し、追加試験を中止した。
- 肘ID1とC610 ID4の能動移動試験は未実施。現在の自動保持版を正常動作確認済みと扱わないこと。

## 統合元・状態

- 作業ブランチ: `Yasaki_Control_dev`、作業開始HEAD `b047b28`。
- リモート照合・fetchした最新 `origin/ohmori/service_dev`: `a259b99aa54d5563ff7e75dd9694fa5cbfdf9609`（update rotation）。
- 共通祖先 `628b34d948feb7722df6860617782a83a5b158bc` からの8ファイルの変更を、既存の未コミットDOB/MPC変更と三者比較して内容統合。マージコミットは作成していない。
- 未コミット・未Push。実機なしで作成。書き込み、リセット、ROS操作、実機試験は実施していない。
- 統合直前バックアップ: `/tmp/catch2026-integration.5mI0SS/before-integration.tar.gz`（`.git`、build、debug_logsを除外。永続保管ではない）。

## 構成と採用値

| バス | モーター | 制御・採用設定 | 校正入力 |
| --- | --- | --- | --- |
| CAN2 | C610 ID4 | 調整済みDOB＋Mode5 MPC、10A、MPC速度825mm/s | Sensor2、NO |
| CAN2 | C610 ID1 | 既存制御、2A。MPC未調整 | Sensor1、NO |
| CAN2 | C620 ID3 | DOB＋Mode5 MPC、20A、200mm/s、加減速度100000mm/s² | Sensor3=PA5、NO、High作動 |
| CAN3 | Robstride02 ID2（根元） | DOB＋Mode5 MPC、5A、400deg/s | 既存オフセット8deg |
| CAN3 | Robstride05 Edu ID1（肘） | DOB＋Mode5 MPC、5A、400deg/s | 既存オフセット67deg |

C620の一次遅れ時定数は33.33ms、DOB帯域5、速度PIはP=0.1/I=0。校正5A・伸長探索7Aは通常運転20Aと別設定。目標到達の加減速制限は100000mm/s²だが、機構端に対する制動包絡200mm/s²は従来どおり維持する。

Robstrideは一次遅れ時定数100ms、速度LPF時定数40ms、DOB帯域3。根元P=6/I=12/J=0.607、肘P=8/I=20/J=0.121497を維持。MPC更新20ms、内側DOB更新2ms。通常PPの速度上限15deg/sとは別設定。

C610 ID4の調整済み10Aは維持し、service_devの4Aでは上書きしなかった。通常位置PIDはservice_devのP=2/I=0.01/D=0.2を取り込み、MPC調整値は保持した。

## 起動・運転時の注意

- 起動モードは調整済み4台（Robstride ID1・ID2、C610 ID4、C620 ID3）がMode5。C610 ID1だけMode3を維持する。RoboMasterは校正中のみ速度制御になり、完了後に元の位置モードへ復帰してEnableする。
- **C610 ID4・ID1はEthernet準備・各FB受信後に自動校正を開始する。通電後に動く構成。** 校正速度は−40mm/s。両軸とも校正成功後Enableで原点保持。ID1の実機校正完了は未確認。
- Robstrideは起動時Disableで待機し、初回ROS指令で自動Enableする。初回指令以後の停止時は明示Enableで再開する。
- C620は起動後1秒以上かつ新鮮なCAN FB取得後、一度だけ5A・−10mm/s（500msランプ）で校正し、確認済み範囲を復元して原点保持する。明示Disableで自動開始を取り消し、通信復帰・失敗後の自動再始動はしない。確認済みの同一機構が前提で、機構変更時は固定端の再確認が必要。
- Robstrideの初期接続待ちは2台とも接続される前提。接続待ちと5台同時運転時の負荷・通信は実機で別途確認する。

## service_devとの統合点

- 指令世代番号と無効化、世代照合付き目標送信を取り込み、DOB/MPCの周期更新にも適用。
- PP目標の近傍回転への変換、位置範囲ガード、Enable前の保持目標確認、連続位置追跡を取り込み。
- Enable中はPPを含めType2 FBを使用し、Disable中のみType17ポーリング。遅着Type17で運転中の主位置FBを巻き戻さない。
- ローカルのDOB/MPC、CAN送信結果チェック、電流クリップ、診断、明示Enableと停止経路を保持。
- CAN2はID1/ID3/ID4を単一0x200フレームへ集約。C620専用サービスを他IDが呼ぶことを拒否する。
- LwIP専用ヒープ `0x20078000` から8KBの配置修正を保持。

## オフライン検証

実行方法:

```bash
bash tools/test_integrated_firmware.sh
cmake --build --preset Debug
cmake --build --preset Release
python3 tools/check_lwip_layout.py build/Debug/26catch_f7.elf build/Release/26catch_f7.elf
```

- ホスト試験すべてPASS: CAN2混在フレーム、個別Disable、C620通信途絶時に他軸出力を維持、ID4/C620同時MPC計算、C620自動校正・開始取消を含む22シナリオ、ID4/C620校正後Mode5復帰とDisable維持、各MPC公称モデル、Robstride速度推定と採用5A DOB制限。
- Debug/Releaseビルド成功、LwIP領域非重複チェックPASS。
- Debug RAM434976B（88.50%）、Flash282852B。Release RAM434920B（88.48%）、Flash200912B。LwIP専用8KBは別領域。
- ホスト公称モデル試験は実機性能の保証ではない。**5台同時の周期達成、スタック余裕、CAN負荷、サービス中の競合、実機挙動は未検証。** 実機復帰後に計測する。

成果物: `build/Debug/26catch_f7.elf`、`build/Release/26catch_f7.elf`。

## 実機接続後の初回確認（同日14:00～14:03 JST）

ユーザーの書き込み・試験指示を受け、上記オフライン版を実機に書き込んだ。

- ST-LINK `066DFF363445503043032152`、NUCLEO-F767ZIにDebug ELFを書き込み・ベリファイ・リセット成功。
- ELF SHA256: `1bb7e7f95befa5ebae7b00904ecf93eba58ed9a549f6da384407473ac933623a`。
- 起動ログ: `flash_boot.jsonl`。C610 ID1/ID4の校正開始とRobstride両軸接続を確認。ログの早期「Calibration done」は開始前にも出るため、これだけで完了判定しない。
- PC有線LANがアドレス未設定だったため、既存の `catch2026-f7-arm-usblan` を有効化（enx9c69d38fde8d、192.168.5.100/24）。agentは既存ROS_DOMAIN_ID=30を使用。
- C620は自動校正済みstate2・Disableを確認。C610 ID4は位置ほぼ0・Disable。C610 ID1は位置−2.50mm・速度0・約−2A・state2だったため明示Disableした。ID1の機構・スイッチ状態は未確認で、正常校正完了とは断定しない。
- 5台の30秒停止通信試験PASS: `../arm_passive_20260910_140206/`。3006メッセージ、最大FB間隔15ms、Disableサービス297回成功、接続異常なし。
- C620承認済み範囲を復元後、20A・速度上限200mm/s・Mode5で0→20mmの単軸試験PASS。到達判定0.897秒（安定窓を含む）、保持位置20.0585mm、fault0。Disable後は18.5004mmに戻った。
- C620試験ログ: `../c620_mode5_20260910_140304/`。最終FBで5台ともstate0・接続code1を確認。
- 今回の能動移動試験はC620のみ。Robstride/C610 ID4の統合版での移動と4軸同時MPC負荷は未試験。C610 ID1の起動時挙動を実機で確認してから次の試験へ進む。

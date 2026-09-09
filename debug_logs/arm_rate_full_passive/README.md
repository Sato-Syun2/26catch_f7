# F7内500Hz FB取得試験（2026-09-09）

追記：手動Launch停止後の同時制御試験は [../arm_rate_full_active/README.md](../arm_rate_full_active/README.md) を参照。
同時運転では過負荷を確認したため、現在の書き込み済み設定はFB100Hzに復旧済み。以下は受信のみの試験時点の履歴。

作業ブランチ: Yasaki_Control_dev。stash 96515851d58a73f96ea2bc91a47a9db0f8596880からアーム作業を復元。
stashは削除せず保持。今回の変更は未コミット・未Push。

## 書き込み済み内容

- arm_test_config.h: ARM_TEST_FB_DIVIDER=1。制御タスク2ms毎に両IDのmechPos/mechVel/iqfをType17で要求。
- 電流ゲインは両IDともKp=.17、Ki=.012、filter=.10で据え置き。
- RoboMaster校正/Enable禁止、アーム角度±145度保護は維持。
- 送信受付・Type17受信・Type2受信・制御周期の1秒集計をarm_rate_logに保存。追加の周期UART出力はなし。
- ELF SHA256: 2289c5da767327e101bbeb600d83afe2a92e6023e589224922a230bc6d7e0ee0
- Debug/Releaseビルド成功、書き込みverify成功。起動ログ ../arm_500hz_full_boot.jsonl。

## 確認済み

Disable状態の安定区間で、両IDともmechPos/mechVel/iqfの新着受信が各500回/秒。
制御タスク500回/秒、通常の最大周期2ms。サービス等に起因する長い周期を含む窓は別扱い。
位置・速度・電流が各500回読めることと、電流指令500回も同時送信できることは区別する。
rate.csv / rate.binはSWD HotPlugで停止指令後に取得。MCUリセットはしていない。
tx_completeはHAL送信完了コールバックの回数、target1/2はHAL送信受付回数であり、各目標のモーター側実行確認ではない。

## 同時制御は未確認

tools/arm_small_step.py --move --velocity-motor 1 --dob --both-axes --amplitude 1で両軸DOBを試行。
初回184705はROS初期FB待ちで中断。184731はmode設定後・Enable前に他publisherを検出して中断。
どちらも最後に両軸Disable成功。制御中500Hzを達成したとは言えない。
競合元（/uros_f7_commandのpublisher）：dof2_to_embedded、handover_action、shoot_action、disturbance_action。
手動操作Launchは勝手に停止していない。競合元停止の確認後に試験を再開する。
ROSへのFB公開周期は従来10msのまま。今回の500HzはF7内部のCAN受信の話。

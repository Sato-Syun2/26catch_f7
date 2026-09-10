#ifndef ARM_TEST_CONFIG_H
#define ARM_TEST_CONFIG_H

/* アーム同定中はRoboMasterの校正・Enable・CAN電流出力を禁止する。
 * ゲインとMPC設定は保存したまま、試験終了後にこのスイッチで復旧する。 */
#define ARM_TEST_DISABLE_ROBOMASTER 0 /* 統合構成でRoboMasterも使用する。 */
/* 校正成功後の位置保持。ホスト試験では旧手動手順との比較用に上書き可能。 */
#ifndef INTEGRATED_STARTUP_HOLD
#define INTEGRATED_STARTUP_HOLD 1
#endif
/* Disable中のType17取得周期。5=100Hz。Enable中は全モードType2を使用。 */
#define ARM_TEST_FB_DIVIDER 5U
#define ARM_TEST_POSITION_MIN_DEG (-145.0f)
#define ARM_TEST_POSITION_MAX_DEG (145.0f)

#endif

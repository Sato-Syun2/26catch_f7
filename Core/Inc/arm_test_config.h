#ifndef ARM_TEST_CONFIG_H
#define ARM_TEST_CONFIG_H

/* アーム同定中はRoboMasterの校正・Enable・CAN電流出力を禁止する。
 * ゲインとMPC設定は保存したまま、試験終了後にこのスイッチで復旧する。 */
#define ARM_TEST_DISABLE_ROBOMASTER 1
/* 通常/DisableのType17取得周期。5=100Hz。Enable中DOBはType2のみ500Hz。 */
#define ARM_TEST_FB_DIVIDER 5U
#define ARM_TEST_POSITION_MIN_DEG (-145.0f)
#define ARM_TEST_POSITION_MAX_DEG (145.0f)

#endif

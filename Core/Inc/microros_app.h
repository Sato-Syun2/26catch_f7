/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    microros_app.h
  * @brief   micro-ROS アプリケーションの公開インターフェース
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef MICROROS_APP_H
#define MICROROS_APP_H

/* ROS指令topicを監視し、一定時間更新がなければ速度／電流制御をDisableする閾値。 */
#define MICROROS_COMMAND_TIMEOUT_MS 100U

#ifdef __cplusplus
extern "C" {
#endif

void MicroRosTask_Run(void);

/* RobstrideTask から呼び出し、受信済みの最新 ROS 指令を CAN へ適用する。 */
void MicroRos_ApplyPendingRobstrideCommands(void);

/* RobstrideTaskから呼び出し、目標値をSetTarget()へ制御周期ごとに渡す。 */
void MicroRos_RefreshRobstrideTargets(void);

/* 各モーター制御タスクから呼び出し、ROS指令のタイムアウトを監視する。 */
void MicroRos_CheckRobstrideCommandTimeout(void);
void MicroRos_CheckRobomasCommandTimeout(void);

/* 高負荷時の受信集約・CAN送信状態を1秒周期でシリアルへ通知する。 */
void MicroRos_ReportDiagnostics(void);

#ifdef __cplusplus
}
#endif

#endif /* MICROROS_APP_H */

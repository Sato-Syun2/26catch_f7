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

#ifdef __cplusplus
extern "C" {
#endif

void MicroRosTask_Run(void);

/* RobstrideTask から呼び出し、受信済みの最新 ROS 指令を CAN へ適用する。 */
void MicroRos_ApplyPendingRobstrideCommands(void);

/* RobstrideTask から定期的に呼び出し、最新の ROS 指令を CAN へ再送する。 */
void MicroRos_RefreshRobstrideTargets(void);

/* 高負荷時の受信集約・CAN送信状態を1秒周期でシリアルへ通知する。 */
void MicroRos_ReportDiagnostics(void);

#ifdef __cplusplus
}
#endif

#endif /* MICROROS_APP_H */

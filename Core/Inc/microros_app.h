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

/* RobstrideTaskから呼び出し、目標値をSetTarget()へ制御周期ごとに渡す。 */
void MicroRos_RefreshRobstrideTargets(void);

/* 高負荷時の受信集約・CAN送信状態を1秒周期でシリアルへ通知する。 */
void MicroRos_ReportDiagnostics(void);

#ifdef __cplusplus
}
#endif

#endif /* MICROROS_APP_H */

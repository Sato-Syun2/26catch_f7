#ifndef C620_COMMISSION_H
#define C620_COMMISSION_H
#include "CAN_RoboMas.h"

/* 統合版は起動後、新鮮なFBを確認して一度だけ自動校正する。 */
#ifndef C620_AUTO_CALIBRATE_ON_BOOT
#define C620_AUTO_CALIBRATE_ON_BOOT 1
#endif

#define C620_CALIBRATION_CURRENT_A 5.0f
#define C620_CALIBRATION_TIMEOUT_MS 60000U
#define C620_CALIBRATION_STALL_MS 3000U
#define C620_TIMEOUT_STOP_SETTLE_MS 500U
#define C620_TIMEOUT_STOP_LIMIT_MS 2000U
#define C620_CALIBRATION_RAMP_MS 500U
#define C620_SURVEY_CURRENT_A 7.0f
/* 停止中の明示設定で許可する通常運転の最大電流。校正・探索は据え置き。 */
#define C620_RUN_SPEED_MM_S 400.0f
#define C620_RUN_CURRENT_A 20.0f
/* 2026-09-10: 校正5A/伸長7A試験後にユーザーが機構端を確認。 */
#define C620_CONFIRMED_END 180.145263671875f

/* ID3/M3508専用。統合版は校正成功後に原点保持。端点探索は明示指令のみ。 */
bool C620_Service(RoboMas_DeviceInfo *dev, const char *command, float data,
                  char *message, unsigned capacity);
bool C620_GuardZero(RoboMas_DeviceInfo *dev, const RoboMas_FeedbackData *fb);
void C620_CalibrationCompleted(void);
bool C620_CalibrationTimeoutReady(void);
bool C620_EnableAllowed(void);
bool C620_PositionTargetAllowed(float target);
bool C620_IsSurvey(void);
float C620_CurrentLimit(const RoboMas_DeviceInfo *dev);
float C620_MpcReference(float position, float velocity, float target,
                         float alpha, float dt);
void C620_ResetMpc(void);
void C620_CancelRequest(void);
#endif

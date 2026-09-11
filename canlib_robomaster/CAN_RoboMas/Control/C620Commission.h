#ifndef C620_COMMISSION_H
#define C620_COMMISSION_H
#include "CAN_RoboMas.h"

/* 完成版は1にする。校正停止原因・端点・調整の確認が済むまでは0を維持。 */
#ifndef C620_AUTO_CALIBRATE_ON_BOOT
#define C620_AUTO_CALIBRATE_ON_BOOT 0
#endif

#define C620_CALIBRATION_CURRENT_A 10.0f
#define C620_CALIBRATION_TIMEOUT_MS 10000U
#define C620_CALIBRATION_RAMP_MS 500U
#define C620_SURVEY_CURRENT_A 1.0f

/* ID3/M3508専用。自動校正でも完了後は停止。端点探索は明示指令のみ。 */
bool C620_Service(RoboMas_DeviceInfo *dev, const char *command, float data,
                  char *message, unsigned capacity);
bool C620_GuardZero(RoboMas_DeviceInfo *dev, const RoboMas_FeedbackData *fb);
void C620_CalibrationCompleted(void);
bool C620_EnableAllowed(void);
bool C620_IsSurvey(void);
float C620_CurrentLimit(const RoboMas_DeviceInfo *dev);
float C620_MpcReference(float position, float velocity, float target,
                         float alpha, float dt);
void C620_ResetMpc(void);
void C620_CancelRequest(void);
#endif

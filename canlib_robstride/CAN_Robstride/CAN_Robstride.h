#ifndef CAN_ROBSTRIDE_H
#define CAN_ROBSTRIDE_H

// Includes --------------------------------

#include <CAN_Robstride_Def.h>
#include <CAN_Robstride_System.h>

// Typedefs --------------------------------

typedef void (*DelayFunction_t)(uint32_t);

// Functions --------------------------------

void Robstride_Init(Robstride_DeviceInfo dev_info_array[], uint8_t size);

void Robstride_PresetParameters(Robstride_DeviceInfo *dev_info, DelayFunction_t f_delay);

void Robstride_SetPIDParams(Robstride_DeviceInfo *dev_info, DelayFunction_t f_delay);

void Robstride_SetVelocityLimit(Robstride_DeviceInfo *dev_info, DelayFunction_t f_delay);

void Robstride_SetCurrentLimit(Robstride_DeviceInfo *dev_info, DelayFunction_t f_delay);

void Robstride_SetTorqueLimit(Robstride_DeviceInfo *dev_info);

void Robstride_Calibration(Robstride_DeviceInfo *device_info, float calib_velocity, ROBSTRIDE_SWITCH_TYPE sw_type, GPIO_TypeDef *limit_port, uint32_t limit_pin, DelayFunction_t f_delay);

void Robstride_SetControl(Robstride_DeviceInfo *dev_info, ROBSTRIDE_CTRL_TYPE new_ctrl_type, DelayFunction_t f_delay);

void Robstride_ChangeControl(Robstride_DeviceInfo *dev_info, ROBSTRIDE_CTRL_TYPE new_ctrl_type, DelayFunction_t f_delay);

void Robstride_SetTarget(Robstride_DeviceInfo *device_info, float target_value);

void Robstride_PID_Pos(Robstride_DeviceInfo *device_info, float target_pos, float now_pos);

void Robstride_SetTarget_Operation(Robstride_DeviceInfo *device_info, float target_toque, float target_pos, float target_speed, float kp, float kd);

uint8_t AreAllRobstridesConnected(Robstride_DeviceInfo dev_info_array[], uint8_t size);

void Robstride_WaitForConnect(Robstride_DeviceInfo dev_info_array[], uint8_t size, DelayFunction_t f_delay);

void Robstride_Initialization(Robstride_DeviceInfo dev_info_array[], uint8_t size);

/* Returns 1 when the motor reports the requested state, 0 on timeout. */
uint8_t Robstride_ControlEnable(Robstride_DeviceInfo *dev_info, DelayFunction_t f_delay);

/* Returns 1 when the motor reports the requested state, 0 on timeout. */
uint8_t Robstride_ControlDisable(Robstride_DeviceInfo *dev_info, DelayFunction_t f_delay);

/* Service path: write/read-back verification with priority CAN traffic. */
uint8_t Robstride_ServiceChangeControl(Robstride_DeviceInfo *dev_info,
                                       ROBSTRIDE_CTRL_TYPE new_ctrl_type,
                                       DelayFunction_t f_delay);

float get_target_pos(float now_pos, float target_pos);

void Robstride_CheckActiveReportStatus(Robstride_DeviceInfo *device_info);

void Robstride_RequestAllParameters(Robstride_DeviceInfo *device_info, DelayFunction_t f_delay);

void Robstride_PrintAllParameters(const Robstride_FeedbackData *fb_data);

void Robstride_Debug_Check_All_Parameters(Robstride_DeviceInfo *device_info, DelayFunction_t f_delay);

#endif // CAN_ROBSTRIDE_H

//
// Created by emile on 23/07/13.
//

#ifndef CAN_ROBOMAS_H
#define CAN_ROBOMAS_H

// Includes --------------------------------
#include "CAN_RoboMas_Def.h"
#include "CAN_RoboMas_System.h"

// Typedefs --------------------------------
typedef void (*DelayFunction_t)(uint32_t);

// Function Prototypes --------------------------------
void RoboMas_Init(RoboMas_DeviceInfo dev_info_array[], uint8_t size); // RoboMas_Init(dev[], 2)的なのを想定してる.

void RoboMas_WaitForConnect(RoboMas_DeviceInfo dev_info_array[], uint8_t size, DelayFunction_t f_delay); // 接続を待つ

void RoboMas_SendRequest(RoboMas_DeviceInfo dev_info_array[], uint8_t size, float update_freq_hz, CAN_HandleTypeDef *phcan); // 計算された指令値を実際に送信する

void RoboMas_send_current(RoboMas_DeviceInfo *device_info, float current, CAN_HandleTypeDef *phcan); // 電流値をそのまま送信する

void RoboMas_Calibration(RoboMas_DeviceInfo *device_info, float calib_vel, ROBOMAS_SWITCH_TYPE sw_type, GPIO_TypeDef* limit_port, uint16_t limit_pin, CAN_HandleTypeDef *phcan); // キャリブレーションを開始する（キャリブレーションが終わると、自動的に enable 状態になる）

void RoboMas_ChangeControl(RoboMas_DeviceInfo *dev_info, ROBOMAS_CTRL_TYPE new_ctrl_type); // 制御モード（位置・速度・電流）を変える

void RoboMas_SetTarget(RoboMas_DeviceInfo *device_info, float target_value); // 目標値を設定

void RoboMas_ControlEnable(RoboMas_DeviceInfo *dev_info); // 制御有効化

void RoboMas_ControlDisable(RoboMas_DeviceInfo *dev_info); // 制御無効化（目標/PIDをクリアし、次周期に0電流送信）

bool RoboMas_IsCalibrationEnded(RoboMas_DeviceInfo *dev_info); // キャリブレーションが終わったか否か

void RoboMas_Wait_For_Calib(RoboMas_DeviceInfo *dev_info, DelayFunction_t f_delay); // キャリブレーションが終わるまで待つ

#endif //CAN_ROBOMAS_H

/*
 * CAN_Main.h
 *
 *  Created on: Feb 6, 2020
 *      Author: Eater
 */

//reference https://hsdev.co.jp/stm32-can/

#ifndef _CAN_MAIN_H_
#define _CAN_MAIN_H_

#include "CAN_AirCylinder_Def.h"
#include "CAN_System_Def.h"
#include "MCMD_CAN_Def.h"
#include "CAN_Servo_Def.h"
#include "CAN_Dynamixel_Kondo_Def.h"
#include "CAN_VL53L0X_Def.h"
#include "main.h"

#define AWAKE_CMD (0)
#define FB_CMD (1)


typedef struct{
    uint8_t servo;
    uint8_t air;
    uint8_t mcmd4;
    uint8_t dynamixel_kondo;
    uint8_t vl53l0x;
} NUM_OF_DEVICES;

typedef void (*DelayFunction_t)(uint32_t); // delay 関数の型


void CAN_SystemInit(CAN_HandleTypeDef *_hcan);  // CANの設定と初期化
void CAN_WaitConnect(const NUM_OF_DEVICES *num_of, DelayFunction_t f_delay); // 全ての基板の接続が確認されるまで待つ

void CANLib_WhenTxMailbox0_1_2CompleteCallbackCalled(const CAN_HandleTypeDef *phcan);
void CANLib_WhenTxMailbox0_1_2AbortCallbackCalled(const CAN_HandleTypeDef *phcan);

void CANLib_WhenCANRxFifo0MsgPending(CAN_HandleTypeDef *hcan, const NUM_OF_DEVICES *num_of);
void CANLib_WhenCANRxFifo1MsgPending(CAN_HandleTypeDef *hcan, const NUM_OF_DEVICES *num_of);

////////////////////////////////////////////////////////////////////////////////////////////
////MCMD
void MCMD_ChangeControl(const MCMD_HandleTypedef *hmcmd); // - (can_device)のMCMDのモーターを制御モードにする.
void MCMD_init(const MCMD_HandleTypedef *hmcmd, DelayFunction_t f_delay);          // - (can_device)のMCMDのモーターに(mode)の初期化命令を送信する.
void MCMD_Calib(const MCMD_HandleTypedef *hmcmd);          // - (can_device)のMCMDに原点合わせをさせる.
void MCMD_Wait_For_Calib(const MCMD_HandleTypedef *hmcmd, DelayFunction_t f_delay); // - (can_device)のMCMDの原点合わせ完了を待つ(MCMD3,MCMD4のみ対応).
void MCMD_Control_Enable(const MCMD_HandleTypedef *hmcmd); // - (can_device)のMCMDのモーターを制御モードにする.
void MCMD_Control_Disable(const MCMD_HandleTypedef *hmcmd); // - (can_device)のMCMDのモーターを制御モードを止める.
void MCMD_SetTarget(const MCMD_HandleTypedef *hmcmd, float target); // - (can_device)のMCMDのモーターのPID制御の目標値を変更する.
MCMD_Feedback_Typedef Get_MCMD_Feedback(const CAN_Device *can_device);

////ServoDriver
void ServoDriver_Init(const CAN_Device *can_device, const CANServo_Param_Typedef *param); // Servo基盤のパラメータを初期化する
void ServoDriver_SendValue(const CAN_Device *can_device, float angle); // Servo基盤に目標値を送る

////AirCylinder
void AirCylinder_Init(const CAN_Device *can_device, Air_PortStatus_Typedef param);
void AirCylinder_SendOutput(const CAN_Device *can_device, Air_PortStatus_Typedef param);

////Dynamixel
void Dynamixel_ChangeControl(const CANDynamixel_HandleTypedef *hdynamixel); // - (can_device)のDynamixelのモーターを制御モードにする.
void Dynamixel_init(const CANDynamixel_HandleTypedef *hdynamixel, DelayFunction_t f_delay);          // - (can_device)のDynamixelのモーターに(mode)の初期化命令を送信する.
void Dynamixel_Control_Enable(const CANDynamixel_HandleTypedef *hdynamixel); // - (can_device)のDynamixelのモーターを制御モードにする.
void Dynamixel_Control_Disable(const CANDynamixel_HandleTypedef *hdynamixel); // - (can_device)のDynamixelのモーターを制御モードを止める.
void Dynamixel_SetTarget(const CANDynamixel_HandleTypedef *hdynamixel, float target); // - (can_device)のDynamixelのモーターのPID制御の目標値を変更する.

CANDynamixel_Kondo_Feedback_Typedef Get_Dynamixel_Feedback(const CAN_Device *can_device); // - (can_device)のDynamixelのモーターのFBを取得する.

////Kondo
void Kondo_ChangeControl(const CANKondo_HandleTypedef *hdynamixel); // - (can_device)のKondoのモーターを制御モードにする.
void Kondo_init(const CANKondo_HandleTypedef *hdynamixel, DelayFunction_t f_delay);          // - (can_device)のKondoのモーターに(mode)の初期化命令を送信する.
void Kondo_Control_Enable(const CANKondo_HandleTypedef *hdynamixel); // - (can_device)のKondoのモーターを制御モードにする.
void Kondo_Control_Disable(const CANKondo_HandleTypedef *hdynamixel); // - (can_device)のKondoのモーターを制御モードを止める.
void Kondo_SetTarget(const CANKondo_HandleTypedef *hdynamixel, float target); // - (can_device)のKondoのモーターのPID制御の目標値を変更する.
void Kondo_SetTarget_With_Time(const CANKondo_HandleTypedef *hdynamixel, float target, float time_sec); // - (can_device)のKondoのモーターのPID制御の目標値を変更する. 軌道生成のための目標移動時間も指定する.

CANDynamixel_Kondo_Feedback_Typedef Get_Kondo_Feedback(const CAN_Device *can_device); // - (can_device)のKondoのモーターのFBを取得する.

////VL53L0X
void VL53L0X_ChangeControl(const CANVL53L0X_HandleTypedef *hvl53l0x);                                     // - (can_device)のVL53L0Xを制御モードにする.
void VL53L0X_init(const CANVL53L0X_HandleTypedef *hvl53l0x);                                              // - (can_device)のVL53L0Xに(mode)の初期化命令を送信する.
void VL53L0X_Control_Enable(const CANVL53L0X_HandleTypedef *hvl53l0x);                                    // - (can_device)のVL53L0Xを制御モードにする.
void VL53L0X_Control_Disable(const CANVL53L0X_HandleTypedef *hvl53l0x);                                   // - (can_device)のVL53L0Xの制御モードを止める.
CANVL53L0X_Feedback_Typedef Get_VL53L0X_Feedback(const CAN_Device *can_device);

#endif /* _CAN_MAIN_H_ */

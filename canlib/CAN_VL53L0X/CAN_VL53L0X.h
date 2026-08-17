/*
 * CAN_VL53L0X.h
 *
 *  Created on: 04 01, 2026
 *      Author: Akitomo KURAKU
 */

#ifndef _INC_CAN_VL53L0X_H_
#define _INC_CAN_VL53L0X_H_

// Includes --------------------------------

#include "main.h"
#include "CAN_System_Def.h"
#include "CAN_VL53L0X_Def.h"
#include "VL53L0X.h"

// Defines --------------------------------

#define CAN_RXBUFFER_SIZE (512)

// Structs --------------------------------

typedef struct {
    CANVL53L0X_HandleTypedef vl53l0x_param; // 制御用のパラメータ（CAN_VL53L0X_Def.hで定義）
    VL53L0X_HandleTypedef vl53l0x;          // VL53L0X を動かす為の構造体（VL53L0X.hで定義）
    statInfo_t_VL53L0X distanceStr;         // VL53L0X の情報の構造体（VL53L0X.hで定義）
    float distance;                         // VL53L0X で検出された距離 [m]
    VL53L0X_Status status;                  // VL53L0X 基板のステータス

    uint64_t incorrect_status_cmd_receive_counter_global; // 間違ったコマンドを受け取った数
    uint8_t _is_initialized;                              // initされたか否か
    uint8_t _start_change_ctrl;                           // change_ctrl開始(パラメータ変更開始)
    uint8_t _finish_change_ctrl;                          // change_ctrl終了

    bool need_vl53l0x_initialize; // 要初期化
    bool need_vl53l0x_enable;     // 要測距開始命令
    bool need_vl53l0x_disable;    // 要測距停止命令
} VL53L0X_Ctrl_Typedef;

// Variables --------------------------------

extern VL53L0X_Ctrl_Typedef vl53l0x_ctrl_global;

// Functions --------------------------------

HAL_StatusTypeDef Process_Fifo0Msg(void);
void WhenCANRxFifo0MsgPending(); // Fifo0MsgPendingで呼び出すこと
void WhenCANRxFifo1MsgPending(); // Fifo1MsgPendingで呼び出すこと

void InitCANVL53L0X(CAN_HandleTypeDef *hcan);
void CANVL53L0X_Update(VL53L0X_Ctrl_Typedef *can_vl53l0x); // 測距を実行する関数

uint8_t CANVL53L0X_Send_Feedback_Main(const VL53L0X_Ctrl_Typedef *can_vl53l0x); // Mainにフィードバックを送信する

void CANVL53L0X_Print(const VL53L0X_Ctrl_Typedef *can_vl53l0x); // デバッグ用出力

#endif // _INC_CAN_VL53L0X_H_

/*
 * CAN_Dynamixel_Kondo.h
 *
 *  Created on: 2026/05/22
 *      Author: Akitomo KURAKU
 */

#ifndef _INC_CAN_DYNAMIXEL_KONDO_H_
#define _INC_CAN_DYNAMIXEL_KONDO_H_

// Includes --------------------------------

#include "main.h"
#include "CAN_System_Def.h"
#include "CAN_Dynamixel_Kondo_Def.h"
#include "dynamixel.h"
#include "kondo_b3m.h"
#include "pid.h"

#define CAN_RXBUFFER_SIZE (64)

// Structs --------------------------------

typedef struct {
    // DYNAMIXEL 用の変数たち
    CANDynamixel_HandleTypedef dynamixel_param; // 制御用のパラメータ（CAN_Dynamixel_Def.hで定義）
    Dynamixel_Typedef dynamixel;                // Dynamixelを動かす為の構造体（dynamixel.hで定義）
    Dynamixel_Feedback_Typedef dynamixel_fb;    // Dynamixelからのフィードバックデータの構造体（dynamixel.hで定義）

    // KONDO B3M 用の変数たち
    CANKondo_HandleTypedef kondo_param; // 制御用のパラメータ（CAN_Kondo_Def.hで定義）
    Kondo_Typedef kondo;                // Kondoを動かす為の構造体（kondo.hで定義）
    Kondo_Feedback_Typedef kondo_fb;    // Kondoからのフィードバックデータの構造体（kondo.hで定義）
    bool need_kondo_set_position;
    float target_time;                        // 制御の目標移動時間（軌道生成用）[s]
    float _last_target_pos;                   // Kondo_goalPosition_withTime() で最後に送信完了した目標値
    float kondo_extended_position_offset;     // 「普通の位置」と「拡張された位置」の原点のずれ [deg]
    float kondo_extended_position_calibrated; // offset を利用して補正された「拡張された位置」 [deg]
    float kondo_expos_ctrl_target_vel;        // 拡張位置制御のときの目標速度 [rps]

    // DYNAMIXEL / KONDO B3M 共通の変数たち
    DYNAMIXEL_OR_KONDO dynamixel_or_kondo; // DYNAMIXEL か KONDO か
    Dynamixel_Kondo_Status status;         // Dynamixel_Kondo（基板）のステータス(状態)

    float target_value; // 制御の目標値

    uint64_t incorrect_status_cmd_receive_counter_global; // 間違ったコマンドを受け取った数
    uint8_t _is_initialized;                              // initされたか否か
    uint8_t _start_change_ctrl;                           // change_ctrl開始(パラメータ変更開始)
    uint8_t _finish_change_ctrl;                          // change_ctrl終了

    bool need_dynamixel_kondo_initialize;
    bool need_dynamixel_kondo_enable;
    bool need_dynamixel_kondo_disable;
} Dynamixel_Kondo_Ctrl_Typedef;

// Variables --------------------------------

extern Dynamixel_Kondo_Ctrl_Typedef dynamixel_kondo_ctrl_global[8];

// Functions --------------------------------

HAL_StatusTypeDef Process_Fifo0Msg(void);
void WhenCANRxFifo0MsgPending(); // Fifo0MsgPendingで呼び出すこと
void WhenCANRxFifo1MsgPending(); // Fifo1MsgPendingで呼び出すこと

void InitCANDynamixel_Kondo(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart);
void CANDynamixel_Kondo_Update(Dynamixel_Kondo_Ctrl_Typedef *can_dynamixel_kondo); // モーターを制御する関数

uint8_t CANDynamixel_Kondo_Send_Feedback_Main(Dynamixel_Kondo_Ctrl_Typedef *can_dynamxiel_kondo); // Main にフィードバックを送信する

void CANDynamixel_Kondo_Print(Dynamixel_Kondo_Ctrl_Typedef *can_dynamixel_kondo); // デバッグ用出力

void CANDynamixel_Kondo_UART_Callback(UART_HandleTypeDef *huart); // dynamixel_kondo 用UART送信コールバック

#endif // _INC_CAN_DYNAMIXEL_KONDO_H_

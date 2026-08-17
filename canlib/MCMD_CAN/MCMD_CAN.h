/*
 * MCMD_CAN.h
 *
 *  Created on: 10 14, 2021
 *      Author: Emile
 */
#ifndef _MCMD_CAN_H_
#define _MCMD_CAN_H_

#include "main.h"
#include "CAN_System_Def.h"
#include "MCMD_CAN_Def.h"
#include "encoder.h"
#include "VNH_motor.h"
#include "pid.h"
#include "motor.h"

#define CAN_RXBUFFER_SIZE	(512)

typedef struct{
    MCMD_HandleTypedef mcmd_param; // 制御用のパラメータ
    Enc_HandleTypedef enc; // encoderの構造体
    Motor_Typedef motor;  // motorを動かす為の構造体
    GPIO_TypeDef* limit_port;  // Calib用のスイッチのポート
    uint32_t limit_pin;  // Calib用のスイッチのピン
    MCMD_Status status; // MCMDのステータス(状態)
    uint8_t end_calib;  // キャリブレーションが終了したか否か.1:True, 0:False
    float target_value;  // 制御の目標値
    float duty;
    uint64_t incorrect_status_cmd_receive_counter_global; // 間違ったコマンドを受け取った数
    uint8_t _is_initialized; // initされたか否か
    uint8_t _start_change_ctrl; // change_ctrl開始(パラメータ変更開始)
    uint8_t _finish_change_ctrl; // change_ctrl終了
}Motor_Ctrl_Typedef;


extern Motor_Ctrl_Typedef motor_ctrl_global[2]; // device番号と配列の番号が対応してる.
/* MCMD4
* device 0 : Channel 1
* device 1 : Channel 2
*/


void InitMotorCom(CAN_HandleTypeDef* hcan, Node_Type _node_type);  // MCMDの初期化をする  //vnhとencのピンとタイマーはmainで設定
// MCMD4なら _node_type = NODE_MCMD4


void WhenCANRxFifo0MsgPending();	//Fifo0MsgPendingで呼び出すこと. fifo0に受信した時に割り込みで呼ばれる関数.
void WhenCANRxFifo1MsgPending();	//Fifo1MsgPendingで呼び出すこと. fifo1に受信した時に割り込みで呼ばれる関数.
HAL_StatusTypeDef Process_Fifo0Msg(void);  // たまったメッセージを処理する関数
uint8_t GetID();  // 基板の番号(node_id)を取得する

void CANMotor_Update(Motor_Ctrl_Typedef* can_motor);  // モーターを制御する関数
void MCMD_SetTimupFlag(); // TODO : 未実装

// ここからはパラメータ設定用の関数
void CANMotor_SetRotDir(Motor_Ctrl_Typedef* can_motor, MCMD_DIR dir);
void CANMotor_SetEncDir(Motor_Ctrl_Typedef* can_motor, MCMD_DIR dir);
// ここまでパラメータ設定用の関数

void CANMotor_Send_Feedback_Main(Motor_Ctrl_Typedef* can_motor);  // Mainにフィードバックを送信する

uint8_t CANMotor_Change_Status(Motor_Ctrl_Typedef* can_motor, MCMD_Status next_status);  // 状態遷移可能か判定し,可能なら遷移する
// 状態遷移できた場合は1をできなかった場合は0を返す. MCMDの状態遷移は全てこの関数を用いて行われる

void _CANMotor_Calib(Motor_Ctrl_Typedef* can_motor);  // 内部用(calibする)
void _CANMotor_Enable(Motor_Ctrl_Typedef* can_motor);  // 内部用(MCMDをenableに)
void _CANMotor_Disable(Motor_Ctrl_Typedef* can_motor);  // 内部用(MCMDをdisableに)

void CANMotor_Print(Motor_Ctrl_Typedef* can_motor);  // 色々表示する


// TODO : データ記録用の構造体

// #define RECORDER_LENGTH 500
#define RECORDER_LENGTH 0
typedef struct{
    int16_t data[RECORDER_LENGTH];
    uint16_t id;
    uint16_t size;
    float magnification;
}Recorder;

void init_record(Recorder* recorder, float magnification);
void add_record(Recorder* recorder, float value);
float get_data_record(Recorder* recorder);
uint8_t is_full_record(Recorder* recorder);

// extern Recorder recorder_motor1;

// 定義ここまで

#endif /* F3K8MD_CAN_H_ */

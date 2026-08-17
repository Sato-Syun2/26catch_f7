/*
 * F3k8MD_CAN_Def.h
 *
 *  Created on: 9/25 , 2021
 *      Author: Emile
 */

#ifndef _MCMD_CAN_DEF_H_
#define _MCMD_CAN_DEF_H_

#include "CAN_System_Def.h"
#include <stdint.h>
#include "pid.h"


typedef enum MCMD_CTRL_TYPE{
    MCMD_CTRL_POS = 0,
    MCMD_CTRL_VEL = 1,
    MCMD_CTRL_DUTY = 2,
    MCMD_CTRL_CURRENT=3
}MCMD_CTRL_TYPE;  // MCMDの制御タイプ

typedef enum MCMD_ACCEL_LIMIT{
    ACCEL_LIMIT_ENABLE = 0,
    ACCEL_LIMIT_DISABLE = 1
}MCMD_ACCEL_LIMIT;  // 制限

typedef enum MCMD_FB{
    MCMD_FB_ENABLE = 0,
    MCMD_FB_DISABLE = 1
}MCMD_FB;  // MainマイコンにFeedbackを送るか否か

typedef enum{
    TIMUP_MONITOR_ENABLE = 0,
    TIMUP_MONITOR_DISABLE = 1,
}MCMD_TIMUP;  // タイムアップを設定するか否か

typedef enum{
    MCMD_FB_POS,
    MCMD_FB_VEL,
    MCMD_FB_DUTY,
    MCMD_FB_CURRENT
}MCMD_FB_TYPE;  // MCMDがMainに送るFeedbackの種類

typedef enum{
    MCMD_DIR_FW=0,
    MCMD_DIR_BC=1
}MCMD_DIR;  // モーターの正転方向

typedef enum{
    CALIBRATION_ENABLE=0,
    CALIBRATION_DISABLE=1
}MCMD_CALIB;  // キャリブレーション(位置合わせ)するか否か.

typedef enum{
    LIMIT_SW_NC,  // Normally closed
    LIMIT_SW_NO   // Normally open
}MCMD_LIMIT_TYPE;  // Limitスイッチの種類

typedef enum{
    GRAVITY_COMPENSATION_ENABLE,
    GRAVITY_COMPENSATION_DISABLE,
}MCMD_GRAVITY_COMPENSATION;  // 重力補償をonにするか.
// (たいてい重力補償はアームの腕の角度のcosやsinで計算するが, ここでは制御量の定数倍にしている)

//MCMD→Mainのフィードバック用の定義
typedef enum{
    MCMD_STATUS_INIT,
    MCMD_STATUS_CALIB,
    MCMD_STATUS_ENABLE,
    MCMD_STATUS_DISABLE,
    MCMD_STATUS_CHANGE_CTRL,
}MCMD_Status;  // MCMDの状態

//commands
typedef enum{
    MCMD_CMD_AWAKE = 0,
    MCMD_CMD_FB = 1,
    MCMD_CMD_INIT1,
    MCMD_CMD_INIT2,
    MCMD_CMD_INIT3,
    MCMD_CMD_CHANGE_CTRL1,
    MCMD_CMD_CHANGE_CTRL2,
    MCMD_CMD_CHANGE_CTRL3,
    MCMD_CMD_CHANGE_CTRL4,
    MCMD_CMD_CALIB,
    MCMD_CMD_ENABLE,
    MCMD_CMD_DISABLE,
    MCMD_CMD_SET_TARGET,
} MCMD_Main_CMD;  // Main -> MCMD / MCMD -> Mainの間のコマンド



typedef struct{  // 制御のパラメータ
    MCMD_CTRL_TYPE ctrl_type;           // 制御方法
    PID_StructTypedef PID_param;        // PID制御のパラメータ
    MCMD_ACCEL_LIMIT accel_limit;       // 目標値の急激な変化を抑える
    float accel_limit_size;             // SmoothControlでの目標値の制限値
    MCMD_FB feedback;                   // フィードバックをするか否か
    MCMD_TIMUP timup_monitor;           // 一定時間データ受信がなければモーターをフリーに
    MCMD_GRAVITY_COMPENSATION gravity_compensation;  // 重力補償(制御量の定数倍を入力に加える)をするか否か
    float gravity_compensation_gain;
}MCMD_Control_Param;


typedef struct{  //mcmdのパラメータを保存する構造体
    CAN_Device device;
    MCMD_FB_TYPE fb_type;              // フィードバック＝エンコーダorポット
    MCMD_DIR enc_dir;                  // encoderの向きの正負(MCMD内部でのフィードバック正負)
    MCMD_DIR rot_dir;                  // モーター回転方向正負
    MCMD_LIMIT_TYPE limit_sw_type;     // リミットスイッチのタイプ
    MCMD_CALIB calib;                  // キャリブレーション時原点あわせ
    float calib_duty;                  // キャリブレーション時の出力(duty)
    float offset;                      // リミットスイッチの当たった座標
    float quant_per_unit;              // エンコーダ1パルスあたりの制御量の変化
    MCMD_Control_Param ctrl_param;     // PID制御のパラメータを管理する
}MCMD_HandleTypedef;


typedef struct{
    float value;  // 値
    MCMD_FB_TYPE fb_type;  // feedbackのタイプ
    MCMD_Status status;  // MCMDの状態
    uint8_t end_calib;  // キャリブレーションが終了したか否か.1:True, 0:False
}MCMD_Feedback_Typedef;  // Feedback時に送信する構造体


typedef struct{
    MCMD_Feedback_Typedef feedback_motor[2];
}MCMD_Feeback_Main_typedef;

#endif /* F3K8MD_CAN_DEF_H_ */

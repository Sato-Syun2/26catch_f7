/*
 * CAN_Dynamixel_Kondo_Def.h
 *
 *  Created on: May 13, 2026
 *      Author: Akitomo KURAKU
 */

// Includes --------------------------------

#ifndef INC_CAN_DYNAMIXEL_KONDO_DEF_H_
#define INC_CAN_DYNAMIXEL_KONDO_DEF_H_
#include <stdint.h>
#include "pid.h"

// Enumerates --------------------------------

typedef enum {
    DYNAMIXEL_MODEL_EX_106_PLUS,
    DYNAMIXEL_MODEL_MX_28,
    DYNAMIXEL_MODEL_MX_64,
    DYNAMIXEL_MODEL_MX_106,
    DYNAMIXEL_MODEL_DX_117,
    DYNAMIXEL_MODEL_RX_24F,
    DYNAMIXEL_MODEL_RX_64,
    DYNAMIXEL_MODEL_XM430_W350_R,
    DYNAMIXEL_MODEL_XH430_V350_R,
    DYNAMIXEL_MODEL_H42_20_S300_R,
} Dynamixel_MODEL; // Dynamixel の Model Number

typedef enum {
    DYNAMIXEL_FB_ENABLE = 0,
    DYNAMIXEL_FB_DISABLE = 1
} Dynamixel_FB; // Main マイコンに Feedback を送るか否か

typedef enum {
    // Dynamixel
    DYNAMIXEL_FB_POS, // 位置
    DYNAMIXEL_FB_VEL, // 速度
    DYNAMIXEL_FB_CUR, // 電流
    DYNAMIXEL_FB_MOV, // 動いているかどうか
    DYNAMIXEL_FB_VOL, // Dynamixel の電源電圧
    DYNAMIXEL_FB_PWM, // PWM
    DYNAMIXEL_FB_TMP, // 温度
    DYNAMIXEL_FB_LOA, // 負荷

    // Kondo
    KONDO_FB_EXPOS,                  // 拡張位置（起動時からの累積回転数を算入した位置）
    KONDO_FB_POS = DYNAMIXEL_FB_POS, // 位置
    KONDO_FB_VEL = DYNAMIXEL_FB_VEL, // 速度
    KONDO_FB_CUR = DYNAMIXEL_FB_CUR, // 電流
    KONDO_FB_VOL = DYNAMIXEL_FB_VOL, // Kondo の電源電圧
    KONDO_FB_TMP = DYNAMIXEL_FB_TMP, // 温度

} Dynamixel_Kondo_FB_TYPE; // Dynamixel_Kondo 基板 が Main に送る Feedback の種類

typedef enum {
    DYNAMIXEL_CTRL_POS,   // 位置制御
    DYNAMIXEL_CTRL_VEL,   // 速度制御
    DYNAMIXEL_CTRL_CUR,   // 電流制御
    DYNAMIXEL_CTRL_EXPOS, // 拡張位置制御
    DYNAMIXEL_CTRL_CUPOS, // 電流に基づく位置制御
    DYNAMIXEL_CTRL_PWM,   // PWM 制御
} Dynamixel_CTRL_TYPE;    // Dynamixel の制御タイプ

typedef enum {
    DYNAMIXEL_MOV = 1, // モータが動作中
    DYNAMIXEL_STOP = 0 // モータが停止中

} Dynamixel_MOV; // モータの動作中/停止中

typedef enum {
    DYNAMIXEL_DIR_FW = 0, // 順方向
    DYNAMIXEL_DIR_BC = 1  // 逆方向

} Dynamixel_DIR; // モーターの正転方向

typedef enum {
    DYNAMIXEL_KONDO_STATUS_INIT,
    DYNAMIXEL_KONDO_STATUS_ENABLE,
    DYNAMIXEL_KONDO_STATUS_DISABLE,
    DYNAMIXEL_KONDO_STATUS_CHANGE_CTRL,
} Dynamixel_Kondo_Status; // Dynamixel / Kondo の状態

typedef enum {
    KONDO_FB_ENABLE = 0,
    KONDO_FB_DISABLE = 1
} Kondo_FB; // Main マイコンに Feedback を送るか否か

typedef enum {
    KONDO_CTRL_POS,  // 位置制御
    KONDO_CTRL_VEL,  // 速度制御
    KONDO_CTRL_TOR,  // トルク制御（電流制御に相当）
    KONDO_CTRL_EXPOS // 拡張位置制御（DYNAMIXEL_KONDO 基板上で PID ループを回すことにより \pm 320 deg の範囲に縛られずに制御可能）

} Kondo_CTRL_TYPE; // Kondo の制御タイプ

typedef enum {
    KONDO_TRAJECTORY_NORMAL = 0,
    KONDO_TRAJECTORY_EVEN = 1,
    KONDO_TRAJECTORY_THIRDPOLY = 3,
    KONDO_TRAJECTORY_FORTHPOLY = 4,
    KONDO_TRAJECTORY_FIFTHPOLY = 5,
} Kondo_TRAJECTORY_TYPE; // Kondo の軌道生成タイプ

typedef enum CANDynamixel_Kondo_Main_CMD {
    // カッコ内は中身のデータのbit数（1コマンド当たり64bitまで）
    // コマンドの種類は32種類まで

    DYNAMIXEL_KONDO_CMD_AWAKE = 0,
    DYNAMIXEL_KONDO_CMD_FB = 1, // Dynamixel_Kondo_Feedback_Typedef(48)

    // Dynamixel の場合 / Kondo の場合
    DYNAMIXEL_KONDO_CMD_INIT1,        // d_or_k(8), model(8), id(8), rot_dir(8) / d_or_k(8), id(8)
    DYNAMIXEL_KONDO_CMD_INIT2,        // quant_per_degree(32) / quant_per_degree(32)
    DYNAMIXEL_KONDO_CMD_CHANGE_CTRL1, // vel_ki(32), vel_kp(32) / empty(0)
    DYNAMIXEL_KONDO_CMD_CHANGE_CTRL2, // pos_kd(32), pos_ki(32) / expos_kd(32), expos_ki(32)
    DYNAMIXEL_KONDO_CMD_CHANGE_CTRL3, // pos_kp(32) / expos_kp(32)
    DYNAMIXEL_KONDO_CMD_CHANGE_CTRL4, // ctrl_type(8), feedback(8), fb_type(8) / ctrl_type(8), feedback(8), fb_type(8)
    DYNAMIXEL_KONDO_CMD_CHANGE_CTRL5, // profile_acceleration(32), profile_velocity(32) / trajectory_type(8)

    DYNAMIXEL_KONDO_CMD_ENABLE,
    DYNAMIXEL_KONDO_CMD_DISABLE,
    DYNAMIXEL_KONDO_CMD_SET_TARGET, // float(32) / target(32), time[s](32)
} CANDynamixel_Kondo_CMD;           // CANで送受信するコマンド

typedef enum DYNAMIXEL_OR_KONDO {
    DKBOARD_UNINITIALIZED,
    DKBOARD_INITIALIZED_AS_DYNAMIXEL,
    DKBOARD_INITIALIZED_AS_KONDO,
} DYNAMIXEL_OR_KONDO;

// Structs --------------------------------

typedef struct {                     // 制御パラメータたち
    Dynamixel_CTRL_TYPE ctrl_type;   // 制御方法
    PID_StructTypedef PID_POS_param; // 位置 PID のパラメータ（未実装）
    PID_StructTypedef PID_VEL_param; // 速度 PID のパラメータ（未実装）
    float profile_acceleration;      // プロファイルの最大加速度
    float profile_velocity;          // プロファイルの最大速度
    Dynamixel_FB feedback;           // フィードバックをするか否か
} Dynamixel_Control_Param;

typedef struct {
    CAN_Device device;

    Dynamixel_MODEL model; // Dynamixel の型番
    uint8_t id;            // Dynamixel に設定されている ID

    Dynamixel_Kondo_FB_TYPE fb_type; // どのパラメータを Main に送るのか

    Dynamixel_DIR rot_dir; // 正転／逆転

    float quant_per_degree; // 1°に対して, 何倍の値を使いたいか

    Dynamixel_Control_Param ctrl_param; // 制御パラメータたち（上で定義）
} CANDynamixel_HandleTypedef;

typedef struct {
    // 64bitまで
    float value;                       // 値
    Dynamixel_Kondo_FB_TYPE fb_type;   // feedback のタイプ
    Dynamixel_Kondo_Status status;     // Dynamixel / Kondo の状態
} CANDynamixel_Kondo_Feedback_Typedef; // 実際の Feedback 時に送信する構造体

typedef struct {
    CANDynamixel_Kondo_Feedback_Typedef feedback_dynamixel_kondo[8];
} Dynamixel_Kondo_Feeback_Main_typedef;

typedef struct {                           // 制御パラメータたち
    Kondo_CTRL_TYPE ctrl_type;             // 制御方法
    PID_StructTypedef PID_EXPOS_param;     // 拡張位置制御時の位置 PID のパラメータ（kp: [s^-1], ki: [s^-2], kd: [1]）
    Kondo_FB feedback;                     // フィードバックをするか否か
    Kondo_TRAJECTORY_TYPE trajectory_type; // 軌道生成タイプ
    float velocity_limit;                  // 拡張位置制御時の最大速さ [rps]
    float integral_limit;                  // 拡張位置制御時の積分項の最大絶対値 [rot s]
} Kondo_Control_Param;

typedef struct {
    CAN_Device device;

    uint8_t id; // Kondo に設定されている ID

    Dynamixel_Kondo_FB_TYPE fb_type; // どのパラメータを Main に送るのか

    float quant_per_degree; // 1°に対して, 何倍の値を使いたいか

    Kondo_Control_Param ctrl_param; // 制御パラメータたち（上で定義）
} CANKondo_HandleTypedef;

#endif /* INC_CAN_DYNAMIXEL_KONDO_DEF_H_ */

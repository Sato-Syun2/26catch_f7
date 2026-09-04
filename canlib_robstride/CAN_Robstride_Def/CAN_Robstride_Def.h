//
// Created by emile on 23/07/13.
//

#ifndef _CAN_ROBSTRIDE_DEF_H
#define _CAN_ROBSTRIDE_DEF_H

#include <Robstride_Control.h>
#include "CAN_Robstride/Control/Actuator_VelocityDob.h"
#include "stdint.h"
#include "main.h"

typedef enum {
    ROBSTRIDE_CTRL_OPERATION = 0,
    ROBSTRIDE_CTRL_POS = 1,
    ROBSTRIDE_CTRL_VEL = 2,
    ROBSTRIDE_CTRL_CURRENT = 3,
    ROBSTRIDE_CTRL_VEL_DOB = 4
} ROBSTRIDE_CTRL_TYPE; // 制御タイプ

typedef enum {
    ROBSTRIDE_VELOCITY_LIMIT_ENABLE = 1,
    ROBSTRIDE_VELOCITY_LIMIT_DISABLE = 0
} ROBSTRIDE_VELOCITY_LIMIT; // 制限

typedef enum {
    ROBSTRIDE_CURRENT_LIMIT_ENABLE = 1,
    ROBSTRIDE_CURRENT_LIMIT_DISABLE = 0
} ROBSTRIDE_CURRENT_LIMIT; // 制限

typedef enum {
    ROBSTRIDE_TORQUE_LIMIT_ENABLE = 1,
    ROBSTRIDE_TORQUE_LIMIT_DISABLE = 0
} ROBSTRIDE_TORQUE_LIMIT;

typedef enum {
    ROBSTRIDE_USE_OFFSET_POS_INTERNAL = 0, // 内部で保存された原点を用いる
    ROBSTRIDE_USE_OFFSET_POS_INITIAL = 1,  // 初期位置を原点とする
    ROBSTRIDE_USE_OFFSET_POS_CALIB = 2,    // Calibした後の位置を原点とする
} ROBSTRIDE_USE_OFFSET_POS;

typedef enum {
    ROBSTRIDE_ROT_ACW = 0, // 半時計回り anti-clock-wise
    ROBSTRIDE_ROT_CW = 1   // 時計回り clock-wise
} ROBSTRIDE_ROT;           // 回転方向

typedef enum {
    ROBSTRIDE_SWITCH_NO = 0, // normally open
    ROBSTRIDE_SWITCH_NC = 1, // normally closed
} ROBSTRIDE_SWITCH_TYPE;

typedef struct {
    Robstride_PID_StructTypedef pid;
    ROBSTRIDE_CTRL_TYPE ctrl_type; // Run_ Mode
    ROBSTRIDE_USE_OFFSET_POS use_internal_offset;
    ROBSTRIDE_VELOCITY_LIMIT velocity_limit;
    ROBSTRIDE_CURRENT_LIMIT current_limit;
    ROBSTRIDE_TORQUE_LIMIT torque_limit;
    ROBSTRIDE_ROT rotation;
    float offset_pos;
    float velocity_limit_size; // Limit_Spd [rad/s]
    float current_limit_size;  // Limit_Cur [A]
    float torque_limit_size;   // Limit_Torque
    float quant_per_rot;
    Robstride_Actuator_VelocityDob_Parameters velocity_dob;
    Robstride_Actuator_VelocityDob_State velocity_dob_state;
    // ↓ don't change
    float _target_value;
    float _req_value;
    uint8_t _enable_flag;
} Robstride_Ctrl_StructTypedef;

typedef enum {
    Robstride_02 = 2,
    Robstride_04 = 4,
    Robstride_05_Edu = 5
} Robstride_device;

typedef struct Robstride_DeviceInfo {
    Robstride_device device;
    uint8_t device_id;
    uint8_t master_id;
    CAN_HandleTypeDef *phcan;
    Robstride_Ctrl_StructTypedef ctrl_param;
} Robstride_DeviceInfo;

typedef struct Robstride_FeedbackData {
    uint8_t device_id;
    uint8_t master_id;
    uint8_t mcu_id;
    uint8_t get_flag;
    Robstride_device device;
    int plus_minus;
    float offset_pos;
    float quant_per_rot;
    float position;
    float velocity;
    float current;
    float torque;
    int temperature;

    uint8_t mode_status; // 0:Disable, 1:Calib, 2:Enable

    uint8_t run_mode;     // 0x7005
    float iq_ref;         // 0x7006
    float spd_ref;        // 0x700A
    float limit_torque;   // 0x700B
    float cur_kp;         // 0x7010
    float cur_ki;         // 0x7011
    float cur_filt_gain;  // 0x7014
    float loc_ref;        // 0x7016
    float limit_spd;      // 0x7017
    float limit_cur;      // 0x7018
    float loc_kp;         // 0x701E
    float spd_kp;         // 0x701F
    float spd_ki;         // 0x7020
    float spd_filt_gain;  // 0x7021
    float acc_rad;        // 0x7022
    float vel_max_pp;     // 0x7024 (Position mode PP)
    float acc_set_pp;     // 0x7025 (Position mode PP)
    uint16_t epscan_time; // 0x7026
    uint32_t can_timeout; // 0x7028
    uint8_t zero_sta;     // 0x7029
    float add_offset;     // 0x702B

    // 読み取り専用値
    float vbus; // 0x701C
} Robstride_FeedbackData;

typedef struct robstride_feedback_data_raw {
    uint8_t _get_counter; // dataを受け取った回数 (offset計算用, max:128)
    int64_t _rot_num;     // 回転数
    uint16_t pos;
    uint16_t vel;
    uint16_t torque;
    uint8_t temp;
    uint8_t mcu_id;
} robstride_feedback_data_raw;

#endif // CAN_ROBSTRIDE_DEF_H

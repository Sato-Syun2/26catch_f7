//
// Created by emile on 23/07/13.
//

#ifndef _CAN_ROBOMAS_DEF_H
#define _CAN_ROBOMAS_DEF_H


#include <stdint.h>
#include <stdbool.h>

#include "RoboMas_Control.h"
#include "main.h"


typedef enum {
    ROBOMAS_CTRL_POS = 0,
    ROBOMAS_CTRL_VEL = 1,
    ROBOMAS_CTRL_CURRENT = 2
} ROBOMAS_CTRL_TYPE;  // C620の制御タイプ


typedef enum {
    ROBOMAS_LIMIT_ENABLE = 0,
    ROBOMAS_LIMIT_DISABLE = 1
} ROBOMAS_LIMIT;  // 制限


typedef enum {
    ROBOMAS_USE_OFFSET_POS_DISABLE = 0,
    ROBOMAS_USE_OFFSET_POS_INTERNAL = 1,  // 初期位置を原点とする
    ROBOMAS_USE_OFFSET_POS_CALIB = 2,  // Calibした後の位置を原点とする
} ROBOMAS_USE_OFFSET_POS;  // M3508自体のencoderのoffset処理を行うか


typedef enum {
    ROBOMAS_ROT_ACW = 0,  // 半時計回り anti-clock-wise
    ROBOMAS_ROT_CW = 1  // 時計回り clock-wise
} ROBOMAS_ROT;  // 回転方向


typedef enum {
    ROBOMAS_SWITCH_NO = 0, // normally open, default LOW
    ROBOMAS_SWITCH_NC = 1, // normally closed, default HIGH
} ROBOMAS_SWITCH_TYPE;


typedef enum{
    ROBOMASTER_C610 = 0,
    ROBOMASTER_C620 = 1,
} ROBOMAS_DEVICE_TYPE;


typedef struct {
    RoboMas_PID_StructTypedef pid_pos;
    RoboMas_PID_StructTypedef pid_vel;
    ROBOMAS_CTRL_TYPE ctrl_type;
    ROBOMAS_USE_OFFSET_POS use_internal_offset;
    ROBOMAS_LIMIT current_limit;
    ROBOMAS_LIMIT velocity_limit;
    ROBOMAS_ROT rotation;
    float current_limit_size;
    float velocity_limit_size;
    float quant_per_rot;
    float offset_pos;
    // ↓ don't change
    float _target_value;
    float _req_value;
    volatile bool _target_valid;
    volatile bool _enable_flag;

    // for calibration
    bool _is_calibrating;
    ROBOMAS_SWITCH_TYPE _sw_type;
    GPIO_TypeDef* _limit_port;
    uint16_t _limit_pin;
    ROBOMAS_CTRL_TYPE _ctrl_type_before_calib; // キャリブレーション開始前の制御モード
} RoboMas_Ctrl_StructTypedef;


typedef struct RoboMas_DeviceInfo {
	ROBOMAS_DEVICE_TYPE device_type;
    uint8_t device_id;
    RoboMas_Ctrl_StructTypedef ctrl_param;
} RoboMas_DeviceInfo;


typedef struct RoboMas_FeedbackData {
    uint8_t device_id;
    uint8_t get_flag;
    float position;
    float velocity;
    float current;
} RoboMas_FeedbackData;


typedef struct robomas_feedback_data_raw {
    volatile uint8_t _get_counter; // dataを受け取った回数 (offset計算用, max:128)
    int64_t _rot_num;  //回転数
    uint16_t pos;
    uint16_t _internal_offset_pos;  // encoderの初期位置自体のoffset
    uint16_t pos_pre;

    int16_t vel;
    int16_t cur;
} robomas_feedback_data_raw;


#endif //_CAN_ROBOMAS_DEF_H

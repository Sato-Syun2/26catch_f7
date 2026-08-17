/*
 * CAN_VL53L0X_Def.h
 *
 *  Created on: Apr 01, 2026
 *      Author: Akitomo KURAKU
 */

#ifndef INC_CAN_VL53L0X_DEF_H_
#define INC_CAN_VL53L0X_DEF_H_

// Includes --------------------------------

#include <stdint.h>

#include "CAN_System_Def.h"

// Enumerates --------------------------------

typedef enum {
    VL53L0X_STATUS_INIT,
    VL53L0X_STATUS_ENABLE,
    VL53L0X_STATUS_DISABLE,
    VL53L0X_STATUS_CHANGE_CTRL,
} VL53L0X_Status; // VL53L0X 基板の状態

typedef struct {
    CAN_Device device;
} CANVL53L0X_HandleTypedef;

typedef enum CANVL53L0X_Main_CMD {
    // カッコ内は中身のデータのbit数（1コマンド当たり64bitまで）
    // コマンドの種類は32種類まで

    VL53L0X_CMD_AWAKE = 0,
    VL53L0X_CMD_FB = 1, // MCMD_Feedback_Typedef()

    VL53L0X_CMD_INIT,        // (0)
    VL53L0X_CMD_CHANGE_CTRL, // (0)

    VL53L0X_CMD_ENABLE,
    VL53L0X_CMD_DISABLE,
} CANVL53L0X_CMD; // CANで送受信するコマンド

// Structs --------------------------------

typedef struct {
    // 64bitまで
    float value;               // 距離 [m]
    VL53L0X_Status status;     // VL53L0X 基板の状態
} CANVL53L0X_Feedback_Typedef; // 実際のFeedback時に送信する構造体

typedef struct {
    CANVL53L0X_Feedback_Typedef feedback_vl53l0x;
} VL53L0X_Feeback_Main_typedef;

#endif /* INC_CAN_VL53L0X_DEF_H_ */

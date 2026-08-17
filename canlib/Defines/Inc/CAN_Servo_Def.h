/*
 * ServoDriver_CAN_Def.h
 *
 *  Created on: Jan 11, 2021
 *      Author: yuta2
 */

#ifndef INC_CAN_SERVO_DEF_H_
#define INC_CAN_SERVO_DEF_H_
#include <stdint.h>


typedef struct{
    float pulse_width_min;
    float pulse_width_max;
    float pwm_frequency;
    float angle_range;
    float angle_offset;
}CANServo_Param_Typedef;  // mainからservoにCANで送信するパラメータを持つ構造体


typedef enum CANServo_Main_CMD{
    SERVO_CMD_AWAKE,
    SERVO_CMD_INIT1,
    SERVO_CMD_INIT2,
    SERVO_CMD_INIT3,
    SERVO_CMD_SET_TARGET,
}CANServo_CMD; // CANで送受信するコマンド


#endif /* INC_CAN_SERVO_DEF_H_ */

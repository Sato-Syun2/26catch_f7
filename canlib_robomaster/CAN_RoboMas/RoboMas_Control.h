//
// Created by emile on 23/07/19.
//

#ifndef ROBOMAS_CONTROL_H
#define ROBOMAS_CONTROL_H

#include "stdint.h"
#include "stdbool.h"


typedef struct {
    //gains
    float kp;
    float ki;
    float kd;
    float kff;  // 2dof

    //values
    float _integral;
    float _prev_value;
    bool _output_saturated;
    bool _anti_windup_active;
} RoboMas_PID_StructTypedef;


void RoboMas_PID_Ctrl_init(RoboMas_PID_StructTypedef *params);

float RoboMas_PID_Ctrl(RoboMas_PID_StructTypedef *params, float value_diff, float target_value, float update_freq);

float RoboMas_PID_Ctrl_AW(RoboMas_PID_StructTypedef* params, float value_diff, uint8_t accel_limit_enable, float max_value, float update_freq);

/* POS_AW以外の既存制御で使う、service_dev互換の制限付きPID。 */
float RoboMas_PID_Ctrl_ServiceAW(RoboMas_PID_StructTypedef *params,
                                 float value_diff,
                                 uint8_t accel_limit_enable,
                                 float max_value,
                                 float update_freq);


#endif //ROBOMAS_CONTROL_H

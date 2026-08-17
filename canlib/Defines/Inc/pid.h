/*
 * pid.h
 *
 *  Created on: 2020/02/29
 *      Author: nabeya11
 */

#ifndef PID_H_
#define PID_H_

#include "main.h"

typedef struct{
    //gains
    float kp;
    float ki;
    float kd;
    float kff;

    //values
    float integral;
    float prev_value;
}PID_StructTypedef;

void PID_Ctrl_init(PID_StructTypedef* params);
//float PID_Ctrl(PID_StructTypedef*, float);
float PID_Ctrl(PID_StructTypedef* params, float value_diff, float target_value, float update_freq);

#endif /* PID_H_ */

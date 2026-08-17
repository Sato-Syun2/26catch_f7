//
// Created by emile on 23/07/19.
//

#ifndef ROBSTRIDE_CONTROL_H
#define ROBSTRIDE_CONTROL_H

#include "stdint.h"

typedef struct {
    // gains
    float kp_pos;     // 位置制御Pゲイン
    float kp_vel;     // 速度制御Pゲイン
    float ki_vel;     // 速度制御Iゲイン
    float filter_vel; // 速度制御フィルタ
    float kp_cur;     // 電流制御Pゲイン
    float ki_cur;     // 電流制御Iゲイン
    float filter_cur; // 電流制御フィルタ

    // values
    float _integral;
    float _prev_value;
} Robstride_PID_StructTypedef;

#endif // ROBSTRIDE_CONTROL_H

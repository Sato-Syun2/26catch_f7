//
// Created by emile on 23/07/19.
//

#include "math.h"
#include "RoboMas_Control.h"


float _clip_f(float value, float min, float max){
    return fminf(max, fmaxf(value, min));
}

void RoboMas_PID_Ctrl_init(RoboMas_PID_StructTypedef *params) {
    params->_integral = 0.0f;
    params->_prev_value = 0.0f;
}

float RoboMas_PID_Ctrl(RoboMas_PID_StructTypedef *params, float value_diff, float target_value, float update_freq) {
    params->_integral += (value_diff + (params->_prev_value)) / 2.0f / update_freq; // 積分(台形近似)
    float diff = (value_diff - params->_prev_value);  // 差分
    params->_prev_value = value_diff;
    return (value_diff * params->kp + params->_integral * params->ki + diff * params->kd + target_value * params->kff);
}

float RoboMas_PID_Ctrl_AW(RoboMas_PID_StructTypedef* params, float value_diff, uint8_t accel_limit_enable, float max_value, float update_freq){
    float integral = params->_integral + (value_diff + (params->_prev_value)) / 2.0f / update_freq; // 積分(台形近似)
    float diff = (value_diff - params->_prev_value);  // 差分
    float ans = (value_diff * params->kp + integral * params->ki + diff * params->kd);
    if(accel_limit_enable && fabsf(ans) > max_value){
        params->_integral += (0.0f + (params->_prev_value)) / 2.0f / update_freq;
        ans = _clip_f(value_diff * params->kp + params->_integral * params->ki + diff * params->kd, -max_value, max_value);
    }else{
        params->_integral = integral;
    };
    params->_prev_value = value_diff;
    return ans;
}

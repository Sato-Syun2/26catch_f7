//
// Created by emile on 23/07/19.
//

#include "math.h"
#include "stddef.h"
#include "stdbool.h"
#include "RoboMas_Control.h"


float _clip_f(float value, float min, float max){
    return fminf(max, fmaxf(value, min));
}

void RoboMas_PID_Ctrl_init(RoboMas_PID_StructTypedef *params) {
    if (params == NULL) {
        return;
    }
    params->_integral = 0.0f;
    params->_prev_value = 0.0f;
    params->_output_saturated = false;
    params->_anti_windup_active = false;
}

float RoboMas_PID_Ctrl(RoboMas_PID_StructTypedef *params, float value_diff, float target_value, float update_freq) {
    if (params == NULL || !isfinite(value_diff) ||
        !isfinite(update_freq) || update_freq <= 0.0f) {
        return 0.0f;
    }

    params->_output_saturated = false;
    params->_anti_windup_active = false;

    params->_integral += (value_diff + params->_prev_value) /
                         (2.0f * update_freq);
    const float diff = value_diff - params->_prev_value;
    params->_prev_value = value_diff;
    return (value_diff * params->kp +
            params->_integral * params->ki +
            diff * params->kd +
            target_value * params->kff);
}

float RoboMas_PID_Ctrl_AW(RoboMas_PID_StructTypedef *params,
                          float value_diff,
                          uint8_t accel_limit_enable,
                          float max_value,
                          float update_freq){
    if (params == NULL || !isfinite(value_diff) ||
        !isfinite(update_freq) || update_freq <= 0.0f) {
        return 0.0f;
    }

    const float dt = 1.0f / update_freq;
    const float previous_error = params->_prev_value;
    const float error_difference = value_diff - previous_error;
    const float integral_candidate = params->_integral +
                                     0.5f * (value_diff + previous_error) * dt;
    const float unsaturated_output = value_diff * params->kp +
                                     integral_candidate * params->ki +
                                     error_difference * params->kd;
    const float limit = fabsf(max_value);
    const bool limit_enabled = accel_limit_enable &&
                                isfinite(limit) &&
                                limit > 0.0f;
    const bool saturated = limit_enabled &&
                           fabsf(unsaturated_output) > limit;
    const float integral_drive = params->ki * value_diff;
    const bool drives_further =
        (unsaturated_output > limit && integral_drive > 0.0f) ||
        (unsaturated_output < -limit && integral_drive < 0.0f);

    params->_output_saturated = saturated;
    params->_anti_windup_active = saturated && drives_further;

    /*
     * conditional integration:
     * 飽和をさらに押し込む誤差のときだけ積分を保持し、誤差が反転した
     * ときは積分を巻き戻す。従来の前回誤差の部分加算では、一定誤差
     * でも積分が増え続けるためanti-windupになっていなかった。
     */
    if (fabsf(params->ki) <= 1.0e-12f) {
        params->_integral = 0.0f;
    } else if (!params->_anti_windup_active) {
        params->_integral = integral_candidate;
    }

    const float output = value_diff * params->kp +
                         params->_integral * params->ki +
                         error_difference * params->kd;
    params->_prev_value = value_diff;

    return limit_enabled ? _clip_f(output, -limit, limit) : output;
}

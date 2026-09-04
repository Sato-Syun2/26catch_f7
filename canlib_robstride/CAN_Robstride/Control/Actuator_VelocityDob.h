#ifndef ROBSTRIDE_ACTUATOR_VELOCITY_DOB_H
#define ROBSTRIDE_ACTUATOR_VELOCITY_DOB_H

#include <stdbool.h>

/* Robstride用のF7側速度DOBパラメータ。速度の外部単位はrad/sへ変換して扱う。 */
typedef struct {
    float J;
    float d;
    float K_tau;
    float dob_bandwidth;
    float velocity_kp;
    float velocity_ki;
    float velocity_kd;
    float reference_alpha;

    /* velocity_limitはrad/s、current_limitはA、torque_limitはNm。 */
    float velocity_limit;
    float current_limit;
    float torque_limit;
    /* falseの場合は無効。trueの場合は0.0も有効。torque_limitの負値は不正。 */
    bool velocity_limit_enable;
    bool current_limit_enable;
    bool torque_limit_enable;
    float velocity_unit_to_rad_s;
    float control_period;
} Robstride_Actuator_VelocityDob_Parameters;

typedef struct {
    /* 一次遅れreference modelの状態 [rad/s] */
    float omega_model;

    /* DOBのQ-filter状態 */
    float dob_q_torque;
    float dob_q_velocity;

    /* PI/PID状態 */
    float integral;
    float previous_error;
    float previous_current;

    /* 実験・デバッグ用の計算結果 */
    float disturbance_estimate;
    float feedforward_torque;
    float feedback_torque;
    float dob_torque;
    float unsaturated_torque;
    float final_torque;
    float final_current;
    float dob_beta;
    float reference_beta;

    bool initialized;
    bool output_saturated;
    bool anti_windup_active;
} Robstride_Actuator_VelocityDob_State;

void Robstride_Actuator_VelocityDob_Reset(
    Robstride_Actuator_VelocityDob_State *state);

/* 戻り値はF7から送る電流指令[A]。 */
float Robstride_Actuator_VelocityDob_Update(
    const Robstride_Actuator_VelocityDob_Parameters *parameters,
    Robstride_Actuator_VelocityDob_State *state,
    float velocity_ref,
    float velocity_measured,
    float dt);

#endif /* ROBSTRIDE_ACTUATOR_VELOCITY_DOB_H */

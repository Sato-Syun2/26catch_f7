#include "Actuator_VelocityDob.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

#define ACTUATOR_VELOCITY_DOB_MIN_LIMIT (1.0e-6f)
#define ACTUATOR_VELOCITY_DOB_MAX_EXPONENT (80.0f)

static float clip_symmetric(const float value, const float limit)
{
    return fmaxf(-limit, fminf(value, limit));
}

/* 一次遅れフィルタの厳密なZOH離散化係数。 */
static float exact_lpf_beta(const float bandwidth, const float dt)
{
    const float exponent = bandwidth * dt;

    if (!isfinite(exponent) || exponent >= ACTUATOR_VELOCITY_DOB_MAX_EXPONENT) {
        return 0.0f;
    }

    return expf(-exponent);
}

static bool parameters_are_valid(
    const Robstride_Actuator_VelocityDob_Parameters *const parameters,
    const float dt)
{
    return parameters != NULL &&
           isfinite(dt) && dt > 0.0f &&
           isfinite(parameters->J) && parameters->J > 0.0f &&
           isfinite(parameters->d) && parameters->d >= 0.0f &&
           isfinite(parameters->K_tau) && parameters->K_tau > 0.0f &&
           isfinite(parameters->dob_bandwidth) &&
           parameters->dob_bandwidth >= 0.0f &&
           isfinite(parameters->velocity_kp) &&
           isfinite(parameters->velocity_ki) &&
           isfinite(parameters->velocity_kd) &&
           isfinite(parameters->reference_alpha) &&
           parameters->reference_alpha >= 0.0f &&
           isfinite(parameters->velocity_reference_limit) &&
           (!parameters->velocity_reference_limit_enable ||
            parameters->velocity_reference_limit >
                ACTUATOR_VELOCITY_DOB_MIN_LIMIT) &&
           isfinite(parameters->current_limit) &&
           (!parameters->current_limit_enable ||
            parameters->current_limit > ACTUATOR_VELOCITY_DOB_MIN_LIMIT) &&
           isfinite(parameters->torque_limit) &&
           parameters->torque_limit >= 0.0f &&
           isfinite(parameters->velocity_unit_to_rad_s) &&
           parameters->velocity_unit_to_rad_s > 0.0f &&
           isfinite(parameters->control_period) &&
           parameters->control_period > 0.0f;
}

void Robstride_Actuator_VelocityDob_Reset(
    Robstride_Actuator_VelocityDob_State *const state)
{
    if (state == NULL) {
        return;
    }

    *state = (Robstride_Actuator_VelocityDob_State){0};
}

float Robstride_Actuator_VelocityDob_Update(
    const Robstride_Actuator_VelocityDob_Parameters *const parameters,
    Robstride_Actuator_VelocityDob_State *const state,
    const float velocity_ref,
    const float velocity_measured,
    const float dt)
{
    if (state == NULL || !parameters_are_valid(parameters, dt) ||
        !isfinite(velocity_ref) || !isfinite(velocity_measured)) {
        Robstride_Actuator_VelocityDob_Reset(state);
        return 0.0f;
    }

    const float velocity_reference_limit =
        parameters->velocity_reference_limit;
    const float current_limit = parameters->current_limit;
    const float omega_ref_unlimited = velocity_ref *
                                      parameters->velocity_unit_to_rad_s;
    const float omega_measured = velocity_measured *
                                 parameters->velocity_unit_to_rad_s;
    if (!isfinite(omega_ref_unlimited) || !isfinite(omega_measured)) {
        Robstride_Actuator_VelocityDob_Reset(state);
        return 0.0f;
    }

    const float omega_ref = parameters->velocity_reference_limit_enable
                                ? clip_symmetric(omega_ref_unlimited,
                                                  velocity_reference_limit)
                                : omega_ref_unlimited;
    const bool first_update = !state->initialized;

    if (first_update) {
        /*
         * 起動時の基準モデル/DOBの過渡応答と微分キックを抑える。
         * previous_currentはまだ実際に適用した指令がないため0とする。
         */
        state->omega_model = omega_measured;
        state->dob_q_velocity = omega_measured;
        state->dob_q_torque = 0.0f;
        state->previous_error = 0.0f;
        state->previous_current = 0.0f;
        state->initialized = true;
    }

    /*
     * 一次遅れ基準モデルを厳密なZOH係数で更新する。
     * フィードフォワードの加速度も更新後のomega_modelから計算し、
     * 速度と加速度の評価時刻をそろえる。
     */
    float acceleration_model = 0.0f;
    if (parameters->reference_alpha > 0.0f) {
        const float model_before = state->omega_model;
        state->reference_beta =
            exact_lpf_beta(parameters->reference_alpha, dt);
        const float omega_model_new =
            state->reference_beta * model_before +
            (1.0f - state->reference_beta) * omega_ref;
        if (!isfinite(omega_model_new)) {
            Robstride_Actuator_VelocityDob_Reset(state);
            return 0.0f;
        }

        state->omega_model = omega_model_new;
        acceleration_model = parameters->reference_alpha *
                             (omega_ref - omega_model_new);
    } else {
        /* alpha=0は基準モデルを無効にし、指令を直接通す。 */
        state->reference_beta = 0.0f;
        state->omega_model = omega_ref;
        acceleration_model = 0.0f;
    }
    if (!isfinite(acceleration_model)) {
        acceleration_model = 0.0f;
    }

    /*
     * 連続時間DOBの近似実装:
     *   d_hat = Q [ tau - (J s + d) omega ]
     *          = Q(tau) - J*g*(omega - Q(omega)) - d*Q(omega)
     *
     * beta=exp(-g*dt)でQ(s)=g/(s+g)の状態を厳密なZOH係数で更新する。
     * この実装はg*dtが十分小さいことを前提とした連続系近似であり、
     * 完全な離散時間DOBへの再設計は別の変更として扱う。
     * previous_currentは前周期に実際に適用した電流指令、
     * omega_measuredは現周期に取得した速度計測値を表す。
     */
    float disturbance_estimate = 0.0f;
    if (parameters->dob_bandwidth > 0.0f) {
        const float previous_torque = state->previous_current *
                                      parameters->K_tau;
        if (!isfinite(previous_torque)) {
            Robstride_Actuator_VelocityDob_Reset(state);
            return 0.0f;
        }

        state->dob_beta = exact_lpf_beta(parameters->dob_bandwidth, dt);
        state->dob_q_torque = state->dob_beta * state->dob_q_torque +
                              (1.0f - state->dob_beta) * previous_torque;
        state->dob_q_velocity = state->dob_beta * state->dob_q_velocity +
                                (1.0f - state->dob_beta) * omega_measured;
        if (!isfinite(state->dob_q_torque) ||
            !isfinite(state->dob_q_velocity)) {
            Robstride_Actuator_VelocityDob_Reset(state);
            return 0.0f;
        }

        disturbance_estimate = state->dob_q_torque -
                               parameters->J * parameters->dob_bandwidth *
                               (omega_measured - state->dob_q_velocity) -
                               parameters->d * state->dob_q_velocity;
    } else {
        /*
         * DOBを無効にする。Q状態は現在値へ追従させ、再有効化時の
         * 不連続な推定値を抑える。
         */
        state->dob_beta = 1.0f;
        state->dob_q_torque = state->previous_current * parameters->K_tau;
        state->dob_q_velocity = omega_measured;
        if (!isfinite(state->dob_q_torque)) {
            Robstride_Actuator_VelocityDob_Reset(state);
            return 0.0f;
        }
    }
    if (!isfinite(disturbance_estimate)) {
        disturbance_estimate = 0.0f;
    }

    const float feedforward_torque = parameters->J * acceleration_model +
                                     parameters->d * state->omega_model;
    const float error = state->omega_model - omega_measured;
    const float previous_error = first_update ? error : state->previous_error;
    if (!isfinite(feedforward_torque) || !isfinite(error) ||
        !isfinite(previous_error)) {
        Robstride_Actuator_VelocityDob_Reset(state);
        return 0.0f;
    }

    /* D項はフィルタなしの生の差分。kd=0なら微分値を使用しない。 */
    const float error_difference = error - previous_error;
    const float error_derivative = error_difference / dt;
    float derivative_torque = 0.0f;
    if (parameters->velocity_kd != 0.0f) {
        if (!isfinite(error_derivative)) {
            Robstride_Actuator_VelocityDob_Reset(state);
            return 0.0f;
        }
        derivative_torque = parameters->velocity_kd * error_derivative;
        if (!isfinite(derivative_torque)) {
            Robstride_Actuator_VelocityDob_Reset(state);
            return 0.0f;
        }
    }

    const float integral_candidate = state->integral +
                                     0.5f * (error + previous_error) * dt;
    if (!isfinite(integral_candidate)) {
        Robstride_Actuator_VelocityDob_Reset(state);
        return 0.0f;
    }

    const float feedback_candidate = parameters->velocity_kp * error +
                                     parameters->velocity_ki *
                                         integral_candidate +
                                     derivative_torque;
    const float unsaturated_candidate = feedforward_torque +
                                        feedback_candidate +
                                        disturbance_estimate;
    if (!isfinite(feedback_candidate) ||
        !isfinite(unsaturated_candidate)) {
        Robstride_Actuator_VelocityDob_Reset(state);
        return 0.0f;
    }

    /* 有効な上限が複数ある場合は、電流由来とトルク上限の小さい方を使う。 */
    float torque_limit = parameters->current_limit_enable
                             ? current_limit * parameters->K_tau
                             : FLT_MAX;
    if (parameters->torque_limit_enable) {
        /* torque_limit=0.0fも有効な設定値として扱う。 */
        torque_limit = fminf(torque_limit, parameters->torque_limit);
    }
    if (!isfinite(torque_limit) || torque_limit < 0.0f) {
        Robstride_Actuator_VelocityDob_Reset(state);
        return 0.0f;
    }

    const bool saturated = fabsf(unsaturated_candidate) > torque_limit;
    const float integral_drive = parameters->velocity_ki * error;
    if (!isfinite(integral_drive)) {
        Robstride_Actuator_VelocityDob_Reset(state);
        return 0.0f;
    }

    /*
     * 飽和中でも、積分項が飽和をさらに進める場合だけ積分を停止する。
     * 飽和から戻す方向の積分は許可する。
     */
    const bool drives_further =
        (unsaturated_candidate > torque_limit && integral_drive > 0.0f) ||
        (unsaturated_candidate < -torque_limit && integral_drive < 0.0f);
    state->anti_windup_active = saturated && drives_further;
    if (!state->anti_windup_active) {
        state->integral = integral_candidate;
    }

    const float feedback_torque = parameters->velocity_kp * error +
                                  parameters->velocity_ki * state->integral +
                                  derivative_torque;
    const float unsaturated_torque = feedforward_torque + feedback_torque +
                                     disturbance_estimate;
    if (!isfinite(feedback_torque) || !isfinite(unsaturated_torque)) {
        Robstride_Actuator_VelocityDob_Reset(state);
        return 0.0f;
    }

    const float final_torque = clip_symmetric(unsaturated_torque, torque_limit);
    const float final_current = final_torque / parameters->K_tau;
    if (!isfinite(final_torque) || !isfinite(final_current)) {
        Robstride_Actuator_VelocityDob_Reset(state);
        return 0.0f;
    }

    state->previous_error = error;
    state->previous_current = final_current;
    state->disturbance_estimate = disturbance_estimate;
    state->feedforward_torque = feedforward_torque;
    state->feedback_torque = feedback_torque;
    state->dob_torque = disturbance_estimate;
    state->unsaturated_torque = unsaturated_torque;
    state->final_torque = final_torque;
    state->final_current = final_current;
    state->output_saturated = fabsf(unsaturated_torque) > torque_limit;

    return final_current;
}

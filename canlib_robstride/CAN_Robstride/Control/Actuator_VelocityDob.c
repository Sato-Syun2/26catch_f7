#include "Actuator_VelocityDob.h"

#include <math.h>
#include <stddef.h>
#include <float.h>

#define ACTUATOR_VELOCITY_DOB_MIN_LIMIT (1.0e-6f)
#define ACTUATOR_VELOCITY_DOB_MAX_EXPONENT (80.0f)

static float clip_symmetric(const float value, const float limit)
{
    return fmaxf(-limit, fminf(value, limit));
}

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
           isfinite(parameters->velocity_limit) &&
           (!parameters->velocity_limit_enable ||
            parameters->velocity_limit > ACTUATOR_VELOCITY_DOB_MIN_LIMIT) &&
           isfinite(parameters->current_limit) &&
           (!parameters->current_limit_enable ||
            parameters->current_limit > ACTUATOR_VELOCITY_DOB_MIN_LIMIT) &&
           isfinite(parameters->torque_limit) &&
           parameters->torque_limit >= 0.0f &&
           isfinite(parameters->velocity_unit_to_rad_s) &&
           parameters->velocity_unit_to_rad_s > 0.0f;
}

void Robstride_Actuator_VelocityDob_Reset(Robstride_Actuator_VelocityDob_State *const state)
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

    const float velocity_limit = parameters->velocity_limit;
    const float current_limit = parameters->current_limit;
    const float omega_ref_unlimited = velocity_ref *
                                      parameters->velocity_unit_to_rad_s;
    const float omega_measured = velocity_measured *
                                 parameters->velocity_unit_to_rad_s;
    if (!isfinite(omega_ref_unlimited) || !isfinite(omega_measured)) {
        Robstride_Actuator_VelocityDob_Reset(state);
        return 0.0f;
    }
    const float omega_ref = parameters->velocity_limit_enable
                                ? clip_symmetric(omega_ref_unlimited,
                                                  velocity_limit)
                                : omega_ref_unlimited;
    const bool first_update = !state->initialized;

    if (first_update) {
        /* 蛻晏屓縺ｯ迴ｾ蝨ｨ騾溷ｺｦ縺九ｉreference model繧帝幕蟋九＠縲∝ｾｮ蛻・く繝・け繧帝∩縺代ｋ縲・*/
        state->omega_model = omega_measured;
        state->dob_q_velocity = omega_measured;
        state->dob_q_torque = 0.0f;
        state->previous_error = 0.0f;
        state->previous_current = 0.0f;
        state->initialized = true;
    }

    /* omega_model繧弾xact/ZOH逶ｸ蠖薙・荳谺｡驕・ｌ縺ｧ譖ｴ譁ｰ縺吶ｋ縲・*/
    float acceleration_model = 0.0f;
    if (parameters->reference_alpha > 0.0f) {
        const float model_before = state->omega_model;
        state->reference_beta = exact_lpf_beta(parameters->reference_alpha, dt);
        acceleration_model = parameters->reference_alpha *
                             (omega_ref - model_before);
        state->omega_model = state->reference_beta * model_before +
                             (1.0f - state->reference_beta) * omega_ref;
    } else {
        /* alpha=0縺ｯreference model繧堤┌蜉ｹ縺ｫ縺励∝刈騾溷ｺｦFF縺ｯ繧ｼ繝ｭ縺ｫ縺吶ｋ縲・*/
        state->reference_beta = 0.0f;
        state->omega_model = omega_ref;
    }
    if (!isfinite(acceleration_model)) {
        acceleration_model = 0.0f;
    }

    /*
     * d_hat = Q [ tau - (J s + d) omega ]
     *        = Q(tau) - J*g*(omega - Q(omega)) - d*Q(omega)
     *
     * tau縺ｫ縺ｯ蜑榊屓縺ｮ譛邨る崕豬∵欠莉､繧剃ｽｿ縺・ゅ％繧後↓繧医ｊ縲√ワ繝ｼ繝峨え繧ｧ繧｢縺ｮ
     * saturation蠕後↓螳滄圀縺ｫ驕ｩ逕ｨ縺輔ｌ縺溷・蜉帙ｒDOB縺ｮ蜈･蜉帙→縺励《aturation
     * 閾ｪ菴薙ｒ螟紋ｹｱ縺ｨ縺励※陬懷─縺励↑縺・・     */
    float disturbance_estimate = 0.0f;
    if (parameters->dob_bandwidth > 0.0f) {
        const float previous_torque = state->previous_current *
                                      parameters->K_tau;
        state->dob_beta = exact_lpf_beta(parameters->dob_bandwidth, dt);
        state->dob_q_torque = state->dob_beta * state->dob_q_torque +
                              (1.0f - state->dob_beta) * previous_torque;
        state->dob_q_velocity = state->dob_beta * state->dob_q_velocity +
                                (1.0f - state->dob_beta) * omega_measured;
        disturbance_estimate = state->dob_q_torque -
                               parameters->J * parameters->dob_bandwidth *
                               (omega_measured - state->dob_q_velocity) -
                               parameters->d * state->dob_q_velocity;
    } else {
        /* 谺｡蝗樊怏蜉ｹ蛹匁凾縺ｮ蛻晄悄驕取ｸ｡繧呈椛縺医ｋ縺溘ａ縲＿ state縺縺題ｿｽ蠕薙＆縺帙ｋ縲・*/
        state->dob_beta = 1.0f;
        state->dob_q_torque = state->previous_current * parameters->K_tau;
        state->dob_q_velocity = omega_measured;
    }
    if (!isfinite(disturbance_estimate)) {
        disturbance_estimate = 0.0f;
    }

    const float feedforward_torque = parameters->J * acceleration_model +
                                     parameters->d * state->omega_model;
    const float error = state->omega_model - omega_measured;
    const float previous_error = first_update ? error : state->previous_error;
    const float error_difference = error - previous_error;
    const float error_derivative = error_difference / dt;
    const float integral_candidate = state->integral +
                                     0.5f * (error + previous_error) * dt;
    const float feedback_candidate = parameters->velocity_kp * error +
                                     parameters->velocity_ki *
                                         integral_candidate +
                                     parameters->velocity_kd * error_derivative;
    const float unsaturated_candidate = feedforward_torque +
                                        feedback_candidate +
                                        disturbance_estimate;

    float torque_limit = parameters->current_limit_enable
                             ? current_limit * parameters->K_tau
                             : FLT_MAX;
    if (parameters->torque_limit_enable) {
        /* 0.0fも有効値。無効化はtorque_limit_enableだけで指定する。 */
        torque_limit = fminf(torque_limit, parameters->torque_limit);
    }
    if (!isfinite(torque_limit) || torque_limit < 0.0f) {
        Robstride_Actuator_VelocityDob_Reset(state);
        return 0.0f;
    }

    const bool saturated = fabsf(unsaturated_candidate) > torque_limit;
    const float integral_drive = parameters->velocity_ki * error;
    const bool drives_further =
        (unsaturated_candidate > torque_limit && integral_drive > 0.0f) ||
        (unsaturated_candidate < -torque_limit && integral_drive < 0.0f);

    /* conditional integration: 鬟ｽ蜥後ｒ謚ｼ縺苓ｾｼ繧縺ｨ縺阪□縺醍ｩ榊・繧剃ｿ晄戟縺吶ｋ縲・*/
    state->anti_windup_active = saturated && drives_further;
    if (state->anti_windup_active) {
        /* state->integral縺ｯ菫晄戟縺励・・髄縺阪・隱､蟾ｮ縺ｪ繧画ｬ｡縺ｮ蜻ｨ譛溘〒蟾ｻ縺肴綾縺吶・*/
    } else {
        state->integral = integral_candidate;
    }

    const float feedback_torque = parameters->velocity_kp * error +
                                  parameters->velocity_ki * state->integral +
                                  parameters->velocity_kd * error_derivative;
    const float unsaturated_torque = feedforward_torque + feedback_torque +
                                     disturbance_estimate;
    if (!isfinite(unsaturated_torque)) {
        Robstride_Actuator_VelocityDob_Reset(state);
        return 0.0f;
    }
    const float final_torque = clip_symmetric(unsaturated_torque, torque_limit);
    const float final_current = final_torque / parameters->K_tau;

    if (!isfinite(final_current)) {
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

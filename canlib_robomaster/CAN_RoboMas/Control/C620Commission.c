#include "C620Commission.h"
#include "CAN_RoboMas_System.h"
#include "PositionMpc.h"
#include "main.h"
#include "can.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

enum { UNCALIBRATED, CALIBRATING, CALIBRATED, SURVEYING, CONTACT, READY, FAULT };
enum { NONE, CALIBRATE, SURVEY };
static volatile unsigned state, pending;
static volatile bool boot_calibration_consumed;
static uint32_t start_tick, stall_tick;
static float start_position, stall_position;
static float contact_position, maximum_position;
static float current_limit = 2.5f;
static float survey_current = C620_SURVEY_CURRENT_A;
static Id4PositionMpc mpc;

void C620_ResetMpc(void) { Id4PositionMpc_Reset(&mpc); }
void C620_CancelRequest(void)
{
    /* 起動待ち中の明示Disableでも、この起動回の自動始動を取り消す。 */
    boot_calibration_consumed = true;
    if (pending != NONE || state == CALIBRATING || state == SURVEYING) state = FAULT;
    pending = NONE;
}
bool C620_IsSurvey(void) { return state == SURVEYING; }
bool C620_EnableAllowed(void)
{
    return state == CALIBRATING || state == SURVEYING || state == READY;
}
float C620_CurrentLimit(const RoboMas_DeviceInfo *dev)
{
    if (dev->ctrl_param._is_calibrating) return C620_CALIBRATION_CURRENT_A;
    if (state == SURVEYING) return survey_current;
    return fminf(2.5f, current_limit);
}
void C620_CalibrationCompleted(void)
{
    if (state == CALIBRATING) state = CALIBRATED;
}

/* サービスは停止中にのみ受理。始動処理は500Hz制御タスクで行う。 */
bool C620_Service(RoboMas_DeviceInfo *dev, const char *cmd, float data,
                  char *message, unsigned capacity)
{
    if (strcmp(cmd, "c620_switch") == 0) {
        snprintf(message, capacity, "PA4=%u NO_active_high", (unsigned)
                 HAL_GPIO_ReadPin(sensor2_GPIO_Port, sensor2_Pin));
        return true;
    }
    if (strcmp(cmd, "c620_status") == 0) {
        snprintf(message, capacity, "s=%u end=%.2f max=%.2f", state,
                 (double)contact_position, (double)maximum_position);
        return true;
    }
    snprintf(message, capacity, "not stopped/invalid state");
    if (dev->device_id != 3U || dev->device_type != ROBOMASTER_C620 ||
        dev->ctrl_param._enable_flag || pending != NONE || state == FAULT ||
        !isfinite(data)) return false;
    if (strcmp(cmd, "c620_calibrate") == 0 && state == UNCALIBRATED) {
        pending = CALIBRATE;
    } else if (strcmp(cmd, "c620_survey") == 0 &&
               (state == CALIBRATED || state == CONTACT) &&
               data >= 0.0f && data <= 5.0f) {
        /* 既定1A。今回の明示指令に限り最大5A、通常制御の上限は変えない。 */
        survey_current = data == 0.0f ? C620_SURVEY_CURRENT_A : data;
        /* 目視で端ではないと判明した場合の明示再開。原点は保持し、旧候補を破棄。 */
        contact_position = maximum_position = 0.0f;
        pending = SURVEY;
    } else if (strcmp(cmd, "c620_range") == 0 && state == CONTACT &&
               data > 20.0f && data <= contact_position - 5.0f) {
        maximum_position = data;
        state = READY;
    } else if (strcmp(cmd, "c620_kp") == 0 && data >= 0.0f && data <= 1.0f) {
        dev->ctrl_param.velocity_dob.velocity_kp = data;
    } else if (strcmp(cmd, "c620_ki") == 0 && data >= 0.0f && data <= 2.0f) {
        dev->ctrl_param.velocity_dob.velocity_ki = data;
    } else if (strcmp(cmd, "c620_dob") == 0 && data >= 0.0f && data <= 30.0f) {
        dev->ctrl_param.velocity_dob.dob_bandwidth = data;
    } else if (strcmp(cmd, "c620_alpha") == 0 && data >= 1.0f && data <= 20.0f) {
        dev->ctrl_param.velocity_dob.reference_alpha = data;
    } else if (strcmp(cmd, "c620_current") == 0 && data > 0.0f && data <= 2.5f) {
        current_limit = data;
        dev->ctrl_param.current_limit_size = data;
        dev->ctrl_param.velocity_dob.current_limit = data;
        dev->ctrl_param.velocity_dob.torque_limit = data * dev->ctrl_param.velocity_dob.K_tau;
    } else {
        return false;
    }
    snprintf(message, capacity, "accepted");
    return true;
}

static bool stop(RoboMas_DeviceInfo *dev, bool fault)
{
    dev->ctrl_param._is_calibrating = false;
    state = fault ? FAULT : CONTACT;
    RoboMas_ControlDisable(dev);
    return true;
}

bool C620_GuardZero(RoboMas_DeviceInfo *dev, const RoboMas_FeedbackData *fb)
{
    if (dev->device_type != ROBOMASTER_C620) return false;
    const uint32_t now = HAL_GetTick();
    /* 起動後1秒以上経過し、新鮮なCAN受信が確立してから一度だけ校正。
     * 通信復帰や失敗時に繰り返し自動始動しない。既存の時間/電流保護を共用する。 */
    if (C620_AUTO_CALIBRATE_ON_BOOT && !boot_calibration_consumed &&
        state == UNCALIBRATED && pending == NONE && now >= 1000U &&
        fb->get_flag && RoboMas_FeedbackFresh(dev->device_id)) {
        boot_calibration_consumed = true;
        pending = CALIBRATE;
    }
    const unsigned request = pending;
    if (request != NONE) {
        pending = NONE;
        if (!RoboMas_FeedbackFresh(dev->device_id) || !fb->get_flag ||
            !isfinite(fb->position) || !isfinite(fb->velocity)) return stop(dev, true);
        start_tick = stall_tick = now;
        start_position = stall_position = fb->position;
        if (request == CALIBRATE) {
            state = CALIBRATING;
            /* Disturbance実コードはsensor2。IOC: PA4/Input/Pull-upを維持。 */
            RoboMas_Calibration(dev, -10.0f, ROBOMAS_SWITCH_NO,
                               sensor2_GPIO_Port, sensor2_Pin, &hcan2);
        } else {
            state = SURVEYING;
            RoboMas_ChangeControl(dev, ROBOMAS_CTRL_VEL);
            RoboMas_SetTarget(dev, 0.0f);
            RoboMas_ControlEnable(dev);
        }
    }
    if (state == FAULT) return stop(dev, true);
    if (!dev->ctrl_param._enable_flag) {
        if (state == CALIBRATING || state == SURVEYING) return stop(dev, true);
        return true;
    }
    if (!RoboMas_FeedbackFresh(dev->device_id) || !fb->get_flag ||
        !isfinite(fb->position) || !isfinite(fb->velocity) || !isfinite(fb->current) ||
        !isfinite(dev->ctrl_param._target_value) || fabsf(fb->velocity) > 150.0f)
        return stop(dev, true);
    if (state == CALIBRATING || state == SURVEYING) {
        if (now-start_tick >= (state == CALIBRATING ? C620_CALIBRATION_TIMEOUT_MS : 60000U) ||
            fabsf(fb->position-start_position) > 1000.0f)
            return stop(dev, true);
        if (state == CALIBRATING) {
            /* 電流上限10Aの速度PI。速度指令を0.5秒で-10へ。
             * 速度追従を失った場合は停止し、直接10Aを流し続けない。 */
            if (fabsf(fb->velocity) > 30.0f ||
                dev->ctrl_param.ctrl_type != ROBOMAS_CTRL_VEL) return stop(dev, true);
            dev->ctrl_param._target_value = -10.0f *
                fminf(1.0f, (float)(now-start_tick)/C620_CALIBRATION_RAMP_MS);
        }
        if (fabsf(fb->position-stall_position) > 0.15f || fabsf(fb->velocity) > 2.0f) {
            stall_tick = now;
            stall_position = fb->position;
        }
        /* ユーザー指定: 校正は低速/停滞でも継続し、スイッチで完了する。
         * 校正の固着停止は行わない。時間・通信・範囲の保護は維持する。 */
        if (state == SURVEYING) {
            if (fb->position < -2.0f ||
                dev->ctrl_param.ctrl_type != ROBOMAS_CTRL_VEL)
                return stop(dev, true);
            /* 電流上限は明示指定値。低速PIで伸長し、始動時の停滞を端と誤認しない。 */
            dev->ctrl_param._target_value = 40.0f *
                fminf(1.0f, (float)(now-start_tick)/500.0f);
            if (now-start_tick > 1000U && now-stall_tick >= 1000U &&
                fabsf(fb->current) >= 0.8f*survey_current) {
                contact_position = fb->position;
                /* 停滞位置は端候補。サービスで範囲を承認するまで通常制御は禁止。 */
                return stop(dev, fb->position < 5.0f);
            }
        }
        return false;
    }
    if (state != READY || fb->position < -2.0f ||
        fb->position > contact_position + 1.0f) return stop(dev, true);
    /* 端探索後、内側へ戻る指令は許可。端へ向かう速度/電流は停止する。 */
    const ROBOMAS_CTRL_TYPE mode = dev->ctrl_param.ctrl_type;
    const float target = dev->ctrl_param._target_value;
    if ((mode == ROBOMAS_CTRL_POS || mode == ROBOMAS_CTRL_POS_AW || mode == ROBOMAS_CTRL_POS_MPC)
            ? (target < 5.0f || target > maximum_position)
            : ((fb->position <= 5.0f && target < 0.0f) ||
               (fb->position >= maximum_position && target > 0.0f)))
        return stop(dev, true);
    return false;
}

float C620_MpcReference(float x, float v, float target, float alpha, float dt)
{
    if (state != READY) return 0.0f;
    float ref = PositionMpc_UpdateBounded(&mpc, x, v, target, alpha, dt,
                                         5.0f, maximum_position, 10.0f);
    /* 暫定低速・減速度30座標単位/s^2。ID4の機構長・825mm/sは流用しない。 */
    const float lower = -sqrtf(60.0f*fmaxf(0.0f, x-5.0f));
    const float upper = sqrtf(60.0f*fmaxf(0.0f, maximum_position-x));
    return fmaxf(lower, fminf(upper, ref));
}

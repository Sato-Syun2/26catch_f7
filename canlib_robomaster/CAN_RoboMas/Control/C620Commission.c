#include "C620Commission.h"
#include "arm_test_config.h"
#include "CAN_RoboMas_System.h"
#include "PositionMpc.h"
#include "main.h"
#include "can.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

enum { UNCALIBRATED, CALIBRATING, CALIBRATED, SURVEYING, CONTACT, READY, FAULT };
enum { NONE, CALIBRATE, SURVEY, PULSE };
static volatile unsigned state, pending;
/* 1開始FB、2運転FB、3時間/移動量、4校正モード、6伸長範囲/モード、
 * 8通常範囲、9通常速度、10目標、11明示中断。リセットまで原因を保持。 */
static volatile unsigned fault_reason;
static volatile bool boot_calibration_consumed;
static uint32_t start_tick, stall_tick;
static unsigned calibration_timeout_phase;
static uint32_t timeout_stop_tick, timeout_stable_tick;
static float timeout_stop_position;
static bool origin_from_timeout;
static bool origin_from_stall;
static float start_position, stall_position;
static float contact_position, maximum_position;
static float current_limit = C620_RUN_CURRENT_A; /* 統合運用採用値20A。校正は別上限。 */
static float speed_limit = C620_RUN_SPEED_MM_S;
static float calibration_speed = 10.0f;
static float survey_current = C620_SURVEY_CURRENT_A;
static Id4PositionMpc mpc;
static float mpc_output;
/* 高速パルス試験は通常運転とは別に設定する。 */
static unsigned pulse_phase;
static float pulse_speed, pulse_peak, pulse_stop_x, pulse_stop_v, pulse_distance;
static uint32_t pulse_stop_tick, pulse_stable_tick, pulse_stop_ms;
static bool pulse_done;

static void set_speed(RoboMas_DeviceInfo *dev, float speed)
{
    dev->ctrl_param.velocity_limit_size = speed;
    dev->ctrl_param.velocity_dob.velocity_limit = speed *
        dev->ctrl_param.velocity_dob.velocity_unit_to_rad_s;
}

void C620_ResetMpc(void) { Id4PositionMpc_Reset(&mpc); mpc_output=0.0f; }
void C620_CancelRequest(void)
{
    /* 起動待ち中の明示Disableでも、この起動回の自動始動を取り消す。 */
    boot_calibration_consumed = true;
    if (pending != NONE || state == CALIBRATING || state == SURVEYING) {
        state = FAULT; fault_reason = 11;
    }
    pending = NONE;
    pulse_phase = 0;
}
bool C620_IsSurvey(void) { return state == SURVEYING; }
bool C620_PositionTargetAllowed(float target)
{
    return isfinite(target) && target >= 0.0f && target <= maximum_position;
}
bool C620_EnableAllowed(void)
{
    return state == CALIBRATING || state == SURVEYING || state == READY;
}
float C620_CurrentLimit(const RoboMas_DeviceInfo *dev)
{
    if (dev->ctrl_param._is_calibrating) return C620_CALIBRATION_CURRENT_A;
    if (state == SURVEYING) return survey_current;
    return fminf(C620_RUN_CURRENT_A, current_limit);
}
void C620_CalibrationCompleted(void)
{
    if (state == CALIBRATING) {
        origin_from_timeout = calibration_timeout_phase == 2U;
        printf("[C620] calibration complete: %s\r\n",
               origin_from_timeout ? (origin_from_stall ? "stall origin (switch unverified)" : "timeout origin (switch unverified)") : "limit switch");
        calibration_timeout_phase = 0U;
        state = CALIBRATED;
        if (INTEGRATED_STARTUP_HOLD) {
            /* 同一実機で承認済みの範囲を校正成功時だけ復元する。 */
            contact_position = C620_CONFIRMED_END;
            maximum_position = C620_CONFIRMED_END - 5.0f;
            state = READY;
        }
    }
}

bool C620_CalibrationTimeoutReady(void)
{
    return state == CALIBRATING && calibration_timeout_phase == 2U;
}

/* サービスは停止中にのみ受理。始動処理は500Hz制御タスクで行う。 */
bool C620_Service(RoboMas_DeviceInfo *dev, const char *cmd, float data,
                  char *message, unsigned capacity)
{
    /* 統合CANでは他のRoboMaster宛てにC620状態・操作を返さない。 */
    if (dev == NULL || dev->device_type != ROBOMASTER_C620 || dev->device_id != 3U) {
        snprintf(message, capacity, "C620 ID3 only");
        return false;
    }
    if (strcmp(cmd, "c620_origin") == 0) {
        snprintf(message, capacity, "%s", origin_from_timeout ? (origin_from_stall ? "stall_unverified" : "timeout_unverified") : "switch_or_not_calibrated");
        return true;
    }
    if (strcmp(cmd, "c620_switch") == 0) {
        snprintf(message, capacity, "PA5=%u NO_active_high", (unsigned)
                 HAL_GPIO_ReadPin(sensor3_GPIO_Port, sensor3_Pin));
        return true;
    }
    if (strcmp(cmd, "c620_status") == 0) {
        snprintf(message, capacity, "s=%u end=%.2f max=%.2f", state,
                 (double)contact_position, (double)maximum_position);
        return true;
    }
    if (strcmp(cmd, "c620_fault") == 0) {
        snprintf(message, capacity, "fault=%u", fault_reason);
        return true;
    }
    if (strcmp(cmd, "c620_pulse_fb") == 0) {
        /* 応答文字列上限32文字に収め、選択値で測定値を分割取得する。 */
        if (data == 1)
            snprintf(message, capacity, "v0=%.2f x0=%.2f", (double)pulse_stop_v, (double)pulse_stop_x);
        else if (data == 2)
            snprintf(message, capacity, "d=%.3f ms=%lu", (double)pulse_distance, (unsigned long)pulse_stop_ms);
        else
            snprintf(message, capacity, "p=%u done=%u peak=%.2f", pulse_phase, pulse_done, (double)pulse_peak);
        return true;
    }
    snprintf(message, capacity, "not stopped/invalid state");
    if (dev->device_id != 3U || dev->device_type != ROBOMASTER_C620 ||
        dev->ctrl_param._enable_flag || pending != NONE || state == FAULT ||
        !isfinite(data)) return false;
    if (strcmp(cmd, "c620_pulse") == 0 && state == READY &&
        (fabsf(data) == 200.0f || fabsf(data) == 400.0f || fabsf(data) == 800.0f)) {
        pulse_speed = data;
        pulse_done = false;
        pulse_peak = pulse_stop_v = pulse_stop_x = pulse_distance = 0;
        pulse_stop_ms = 0;
        pending = PULSE;
    } else if (strcmp(cmd, "c620_calibrate") == 0 && state == UNCALIBRATED &&
        data >= 0.0f && data <= 10.0f) {
        calibration_speed = data > 0.0f ? data : 10.0f;
        pending = CALIBRATE;
    } else if (strcmp(cmd, "c620_survey") == 0 &&
               (state == CALIBRATED || state == CONTACT) &&
               data >= 0.0f && data <= C620_SURVEY_CURRENT_A) {
        /* 既定・最大はC620_SURVEY_CURRENT_A。通常制御の上限は変えない。 */
        survey_current = data == 0.0f ? C620_SURVEY_CURRENT_A : data;
        /* 目視で端ではないと判明した場合の明示再開。原点は保持し、旧候補を破棄。 */
        contact_position = maximum_position = 0.0f;
        pending = SURVEY;
    } else if (strcmp(cmd, "c620_restore") == 0 && state == CALIBRATED &&
               data > 20.0f && data <= C620_CONFIRMED_END - 5.0f) {
        /* 再校正後、記録済みの機構端を明示的に復元。自動運転はしない。 */
        contact_position = C620_CONFIRMED_END;
        maximum_position = data;
        state = READY;
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
    } else if (strcmp(cmd, "c620_alpha") == 0 && data >= 1.0f && data <= 60.0f) {
        dev->ctrl_param.velocity_dob.reference_alpha = data;
    } else if (strcmp(cmd, "c620_speed") == 0 && data >= 1.0f && data <= C620_RUN_SPEED_MM_S) {
        /* 明示された段階試験用。起動時はC620_RUN_SPEED_MM_Sを使用する。 */
        speed_limit = data;
        dev->ctrl_param.velocity_limit_size = data;
        dev->ctrl_param.velocity_dob.velocity_limit = data *
            dev->ctrl_param.velocity_dob.velocity_unit_to_rad_s;
    } else if (strcmp(cmd, "c620_current") == 0 && data > 0.0f && data <= C620_RUN_CURRENT_A) {
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

static bool fault(RoboMas_DeviceInfo *dev, unsigned reason)
{
    fault_reason = reason;
    return stop(dev, true);
}

bool C620_GuardZero(RoboMas_DeviceInfo *dev, const RoboMas_FeedbackData *fb)
{
    if (dev->device_type != ROBOMASTER_C620) return false;
    const uint32_t now = HAL_GetTick();
    /* スイッチONなら新鮮なFB取得後の最初の周期で原点確定する。
     * OFFなら従来どおり起動後1秒待って校正を開始する。
     * 通信復帰や失敗時に繰り返し自動始動しない。既存の時間/電流保護を共用する。 */
    if (C620_AUTO_CALIBRATE_ON_BOOT && !boot_calibration_consumed &&
        state == UNCALIBRATED && pending == NONE &&
        (now >= 1000U || HAL_GPIO_ReadPin(sensor3_GPIO_Port, sensor3_Pin)) &&
        fb->get_flag && RoboMas_FeedbackFresh(dev->device_id)) {
        boot_calibration_consumed = true;
        pending = CALIBRATE;
    }
    const unsigned request = pending;
    if (request != NONE) {
        pending = NONE;
        if (!RoboMas_FeedbackFresh(dev->device_id) || !fb->get_flag ||
            !isfinite(fb->position) || !isfinite(fb->velocity)) return fault(dev, 1);
        start_tick = stall_tick = now;
        start_position = stall_position = fb->position;
        if (request == PULSE) {
            if (fabsf(fb->velocity) > 2.0f ||
                !(pulse_speed > 0 ? (fb->position >= 30 && fb->position <= 50) :
                                    (fb->position >= 125 && fb->position <= 145)))
                return fault(dev, 12);
            RoboMas_ChangeControl(dev, ROBOMAS_CTRL_VEL_DOB);
            set_speed(dev, fabsf(pulse_speed));
            RoboMas_SetTarget(dev, 0);
            RoboMas_ControlEnable(dev);
            pulse_phase = 1;
        } else if (request == CALIBRATE) {
            state = CALIBRATING;
            calibration_timeout_phase = 0U;
            origin_from_timeout = false;
            origin_from_stall = false;
            /* Disturbance実コードはsensor3。IOC: PA5/Input/Pull-upを維持。 */
            RoboMas_Calibration(dev, -calibration_speed, ROBOMAS_SWITCH_NO,
                               sensor3_GPIO_Port, sensor3_Pin, &hcan2);
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
        !isfinite(dev->ctrl_param._target_value) ||
        (state != READY && fabsf(fb->velocity) > 150.0f))
        return fault(dev, 2);
    if (state == CALIBRATING || state == SURVEYING) {
        if ((state == SURVEYING && now-start_tick >= 60000U) ||
            fabsf(fb->position-start_position) > 1000.0f)
            return fault(dev, 3);
        if (fabsf(fb->position-stall_position) > 0.15f || fabsf(fb->velocity) >= 2.0f) {
            stall_tick = now;
            stall_position = fb->position;
        }
        if (state == CALIBRATING) {
            /* 校正電流上限はC620_CALIBRATION_CURRENT_A。速度指令を0.5秒で-10へ。
             * 実測速度30mm/s超過による校正中の停止判定は使用しない。 */
            if (dev->ctrl_param.ctrl_type != ROBOMAS_CTRL_VEL) return fault(dev, 4);
            if (calibration_timeout_phase != 0U ||
                now-start_tick >= C620_CALIBRATION_TIMEOUT_MS ||
                now-stall_tick >= C620_CALIBRATION_STALL_MS) {
                /* ユーザー指定: 3秒停滞または時間切れ後、電流ゼロで停止を確認し原点とする。
                 * 途中の引っ掛かりと真の原点は識別できないため完了理由を別記録する。 */
                dev->ctrl_param._target_value = 0.0f;
                if (calibration_timeout_phase == 0U) {
                    origin_from_stall = now-start_tick < C620_CALIBRATION_TIMEOUT_MS;
                    calibration_timeout_phase = 1U;
                    timeout_stop_tick = timeout_stable_tick = now;
                    timeout_stop_position = fb->position;
                    return true;
                }
                if (fabsf(fb->velocity) >= 2.0f ||
                    fabsf(fb->position-timeout_stop_position) > 0.15f) {
                    timeout_stable_tick = now;
                    timeout_stop_position = fb->position;
                }
                if (now-timeout_stable_tick >= C620_TIMEOUT_STOP_SETTLE_MS) {
                    calibration_timeout_phase = 2U;
                    return false; /* 共通の原点登録・位置モード復帰経路へ。 */
                }
                if (now-timeout_stop_tick >= C620_TIMEOUT_STOP_LIMIT_MS) return fault(dev, 3);
                return true; /* 停止待ち中はPIを回さずゼロ電流を送る。 */
            }
            dev->ctrl_param._target_value = -calibration_speed *
                fminf(1.0f, (float)(now-start_tick)/C620_CALIBRATION_RAMP_MS);
        }
        /* 校正は3秒停滞後にゼロ電流で停止確認。探索の端候補判定は別扱い。 */
        if (state == SURVEYING) {
            if (fb->position < -2.0f ||
                dev->ctrl_param.ctrl_type != ROBOMAS_CTRL_VEL)
                return fault(dev, 6);
            /* 電流上限は明示指定値。低速PIで伸長し、始動時の停滞を端と誤認しない。 */
            dev->ctrl_param._target_value = 10.0f *
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
    if (state != READY) return fault(dev, 8);
    if (fabsf(fb->velocity) > fmaxf(30.0f, 1.25f*(pulse_phase ? fabsf(pulse_speed) : speed_limit)))
        return fault(dev, 9);
    if (pulse_phase) {
        const float direction = pulse_speed > 0 ? 1.0f : -1.0f;
        if (dev->ctrl_param.ctrl_type != ROBOMAS_CTRL_VEL_DOB ||
            fb->position < 20 || fb->position > 155 || fabsf(fb->current) > 20.8f)
            return fault(dev, 13);
        pulse_peak = fmaxf(pulse_peak, fabsf(fb->velocity));
        if (pulse_phase == 1 && (now-start_tick >= 400U ||
            direction*(fb->position-start_position) >= 45.0f)) {
            pulse_phase = 2;
            pulse_stop_tick = pulse_stable_tick = now;
            pulse_stop_x = fb->position;
            pulse_stop_v = fb->velocity;
        }
        dev->ctrl_param._target_value = pulse_phase == 1 ? pulse_speed : 0.0f;
        if (pulse_phase == 2) {
            pulse_distance = fmaxf(pulse_distance, direction*(fb->position-pulse_stop_x));
            if (pulse_distance > 40.0f || now-pulse_stop_tick >= 1000U)
                return fault(dev, 14);
            if (fabsf(fb->velocity) >= 2.0f) pulse_stable_tick = now;
            if (now-pulse_stable_tick >= 200U) {
                pulse_stop_ms = pulse_stable_tick-pulse_stop_tick;
                pulse_done = true;
                pulse_phase = 0;
                set_speed(dev, speed_limit);
                RoboMas_ControlDisable(dev);
                return true;
            }
        }
        return false;
    }
    /* 端探索後、内側へ戻る指令は許可。端へ向かう速度/電流は停止する。 */
    const ROBOMAS_CTRL_TYPE mode = dev->ctrl_param.ctrl_type;
    const float target = dev->ctrl_param._target_value;
    /* 原点スイッチ接触中は押し込まず、この周期の電流出力だけをゼロにする。
     * 通常の原点停止でDisableすると、後続の位置指令だけでは再始動できない。
     * READY・Enable・有効な目標を保持し、次の伸長指令を受け付ける。
     * 明示Disableや異常時の停止は従来どおり別経路で処理する。 */
    if (!dev->ctrl_param._startup_hold && mode == ROBOMAS_CTRL_POS_MPC && target == 0.0f &&
        HAL_GPIO_ReadPin(sensor3_GPIO_Port, sensor3_Pin)) {
        dev->ctrl_param._req_value = 0.0f;
        C620_ResetMpc();
        RoboMas_PID_Ctrl_init(&dev->ctrl_param.pid_pos);
        RoboMas_PID_Ctrl_init(&dev->ctrl_param.pid_vel);
        RoboMas_Actuator_VelocityDob_Reset(&dev->ctrl_param.velocity_dob_state);
        return true;
    }
    if ((mode == ROBOMAS_CTRL_POS || mode == ROBOMAS_CTRL_POS_AW || mode == ROBOMAS_CTRL_POS_MPC)
            ? !C620_PositionTargetAllowed(target)
            : ((fb->position <= 5.0f && target < 0.0f) ||
               (fb->position >= maximum_position && target > 0.0f)))
        return true; /* 範囲違反だけではラッチせず、次の内向き指令を受け付ける。 */
    return false;
}

float C620_MpcReference(float x, float v, float target, float alpha, float dt)
{
    if (state != READY) return 0.0f;
    float ref = PositionMpc_UpdateBounded(&mpc, x, v, target, alpha, dt,
                                         0.0f, maximum_position, speed_limit);
    /* 速度目標のステップによる急なトルク立ち上がりを避ける。
     * 端の包絡は最後に適用し、減速側の制限を優先する。 */
    ref=fmaxf(mpc_output-100000.0f*dt,fminf(mpc_output+100000.0f*dt,ref));
    /* 立ち上がり制限だけでは中間目標で減速が遅れるため、目標到達にも包絡を適用。 */
    const float arrival_speed=sqrtf(200000.0f*fabsf(target-x));
    ref=fmaxf(-arrival_speed,fminf(arrival_speed,ref));
    /* 中間目標はMPCに任せるが、機構端の停止包絡200mm/s^2は維持。
     * ID4の機構長・825mm/sは流用しない。 */
    const float lower = -sqrtf(400.0f*fmaxf(0.0f, x));
    const float upper = sqrtf(400.0f*fmaxf(0.0f, maximum_position-x));
    mpc_output=fmaxf(lower, fminf(upper, ref));
    return mpc_output;
}

//
// Created by emile on 23/07/13.
//

// Includes --------------------------------
#include "CAN_RoboMas_Def.h"
#include "CAN_RoboMas_System.h"
#include "CAN_RoboMas.h"
#include "math.h"
#include "stdio.h"
#include "usart.h"
#include "id4_velocity_safety.h"
#include "PositionMpc.h"
#include "arm_test_config.h"

// Private Function Prototypes --------------------------------
static bool get_switch_state(GPIO_TypeDef* limit_port, uint16_t limit_pin, ROBOMAS_SWITCH_TYPE sw_type);
static float _clip_f_abs(float var, float abs_ref);
static int16_t c610_current_f2int(float current);
static int16_t c620_current_f2int(float current);
static void RoboMas_Ctrl_Struct_init(RoboMas_Ctrl_StructTypedef *ctrl_struct);
static bool robomas_is_position_control(ROBOMAS_CTRL_TYPE ctrl_type);

/* 実測機構端を基準とするID4保護。校正後に有効。
 * ハード停止ラッチはモード変更/Disableでも解除せず、再起動のみ。 */
#define ROBOMAS_TEST_GUARD_ID 4U
#define ROBOMAS_TEST_GUARD_MIN_POSITION ID4_HARD_STOP_MIN_MM
#define ROBOMAS_TEST_GUARD_MAX_POSITION ID4_HARD_STOP_MAX_MM
static volatile bool robomas_test_guard_armed = false;
static volatile bool robomas_test_guard_tripped = false;
static volatile bool id4_calibration_completed = false;
static Id4PositionMpc id4_mpc;
static volatile bool id4_mpc_reset_requested = true;
static bool id4_mpc_holding = false;
static volatile uint32_t id4_mpc_max_runtime_ms = 0;
uint32_t Id4PositionMpc_ClockMs(void) { return HAL_GetTick(); }

/* 手動開始する一回限りの端探索。終了/異常後は再起動まで再駆動しない。
 * 停滞は障害物でも生じるため、既知可動位置より手前では端と認定しない。 */
/* 探索完了。通常ビルドでは絶対に自動探索経路を使用しない。 */
static const bool id4_endpoint_survey = false;
static bool id4_survey_started = false;
static bool id4_survey_stopped = false;
static uint32_t id4_survey_start_tick, id4_survey_stall_tick;
static float id4_survey_stall_position;
static volatile float id4_survey_contact_position = NAN;

static bool RoboMas_Id4SurveyZero(RoboMas_DeviceInfo *device,
                                 const RoboMas_FeedbackData *fb)
{
    RoboMas_Ctrl_StructTypedef *c = &device->ctrl_param;
    if (c->_is_calibrating) return false;
    if (id4_survey_stopped) {
        RoboMas_ControlDisable(device);
        return true;
    }
    if (!c->_enable_flag || !c->_target_valid) {
        if (id4_survey_started) id4_survey_stopped = true;
        return true;
    }
    /* 校正後の自動位置保持も行わず、明示的な速度指令だけで開始する。 */
    if (c->ctrl_type != ROBOMAS_CTRL_VEL || c->_target_value <= 0.0f) {
        RoboMas_ControlDisable(device);
        if (id4_survey_started) id4_survey_stopped = true;
        return true;
    }
    const uint32_t now = HAL_GetTick();
    if (!id4_survey_started) {
        id4_survey_started = true;
        id4_survey_start_tick = now;
        id4_survey_stall_tick = now;
        id4_survey_stall_position = fb->position;
    }
    const bool invalid = !isfinite(fb->position) || !isfinite(fb->velocity) ||
        fb->position < -2.0f || fb->position >= 550.0f ||
        fabsf(fb->velocity) > 60.0f || now-id4_survey_start_tick >= 20000U;
    if (fabsf(fb->position-id4_survey_stall_position) > 0.15f ||
        fabsf(fb->velocity) > 2.0f) {
        id4_survey_stall_tick = now;
        id4_survey_stall_position = fb->position;
    }
    const bool stalled = now-id4_survey_start_tick > 500U &&
                         now-id4_survey_stall_tick >= 200U;
    if (invalid || stalled) {
        id4_survey_stopped = true;
        if (!invalid && fb->position >= 513.353f)
            id4_survey_contact_position = fb->position;
        RoboMas_ControlDisable(device);
        printf("[ID4 ENDPOINT] stop x=%.4f v=%.3f current=%.3f contact=%.4f invalid=%u\r\n",
               (double)fb->position, (double)fb->velocity, (double)fb->current,
               (double)id4_survey_contact_position, (unsigned)invalid);
        return true;
    }
    c->_target_value = fminf(c->_target_value, 40.0f);
    return false;
}

/* 高優先度制御経路ではメモリ記録のみ。最大1.024秒、500Hz。 */
typedef struct {
    uint32_t tick;
    float position, velocity, target, model, current, unsaturated;
    uint32_t saturated;
} Id4Diagnostic;
static Id4Diagnostic id4_diag[512];
static volatile uint32_t id4_diag_count = 0U;
static uint32_t id4_diag_read = 0U;
static volatile bool id4_diag_active = false;
static bool id4_diag_started = false;
static volatile bool id4_velocity_braking = false;
static uint32_t id4_brake_stopped_cycles = 0U;

void RoboMas_RequestId4VelocityBrake(void)
{
    id4_velocity_braking = true;
}

void RoboMas_PrintId4Diagnostic(void)
{
    if (id4_diag_active || id4_diag_read >= id4_diag_count) return;
    const Id4Diagnostic d = id4_diag[id4_diag_read];
    char line[160];
    const int length = snprintf(line, sizeof(line),
           "ID4D,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%lu\r\n",
           (unsigned long)d.tick, (double)d.position, (double)d.velocity,
           (double)d.target, (double)d.model, (double)d.current,
           (double)d.unsaturated, (unsigned long)d.saturated);
    /* 一文字ずつのprintfは他タスクと混線するため、一行単位で送信する。
     * UART使用中なら次周期に再試行する。ID4停止時以外は呼び出さない。 */
    if (length > 0 && length < (int)sizeof(line) &&
        HAL_UART_Transmit(&huart3, (uint8_t *)line, (uint16_t)length, 20U) == HAL_OK)
        ++id4_diag_read;
}

static bool RoboMas_SafetyGuardZero(RoboMas_DeviceInfo *device,
                                    const RoboMas_FeedbackData *feedback)
{
    if (device->device_id != ROBOMAS_TEST_GUARD_ID) return false;
    if (id4_endpoint_survey) return RoboMas_Id4SurveyZero(device, feedback);
    /* 起動直後の未確立FB/未校正座標を機構端として判定しない。 */
    if (!id4_calibration_completed) return false;
    if (robomas_test_guard_tripped) {
        RoboMas_ControlDisable(device);
        return true;
    }
    RoboMas_Ctrl_StructTypedef *ctrl = &device->ctrl_param;
    /* 校正中は相対座標が未確定。校正終了後は250mmへの移動を待たず保護。 */
    if (ctrl->_is_calibrating) return false;
    robomas_test_guard_armed = true;
    const bool valid = feedback->get_flag != 0U &&
                       isfinite(feedback->position) && isfinite(feedback->velocity);
    const bool position_mode = robomas_is_position_control(ctrl->ctrl_type);
    const bool outside = !valid ||
        feedback->position <= ROBOMAS_TEST_GUARD_MIN_POSITION ||
        feedback->position >= ROBOMAS_TEST_GUARD_MAX_POSITION;
    /* 速度/電流目標を位置として判定しない。 */
    const bool target_outside = position_mode && ctrl->_target_valid &&
        !id4_position_target_allowed(ctrl->_target_value);
    if (outside || target_outside) {
        robomas_test_guard_tripped = true;
        RoboMas_ControlDisable(device);
        printf("[RoboMas] ID4 safety guard tripped: pos=%.3f mm; current=0\r\n",
               (double)feedback->position);
        return true;
    }
    return false;
}

// Functions --------------------------------
static float _clip_f_abs(float var, float abs_ref) {
    abs_ref = fabsf(abs_ref);
    return fmaxf(fminf(var, abs_ref), -abs_ref);
}

static int16_t c610_current_f2int(float current) {
    return (int16_t) (current * 10000.0f / 10.0f);
}

static int16_t c620_current_f2int(float current) {
    return (int16_t) (current * 16384.0f / 20.0f);
}

static bool robomas_is_position_control(const ROBOMAS_CTRL_TYPE ctrl_type) {
    return ctrl_type == ROBOMAS_CTRL_POS || ctrl_type == ROBOMAS_CTRL_POS_AW ||
           ctrl_type == ROBOMAS_CTRL_POS_MPC;
}

static void RoboMas_Ctrl_Struct_init(RoboMas_Ctrl_StructTypedef *ctrl_struct) {
    ctrl_struct->_target_value = 0.0f;
    ctrl_struct->_req_value = 0.0f;
    ctrl_struct->_target_valid = false;
    ctrl_struct->_enable_flag = false;
    ctrl_struct->_is_calibrating = false;
    RoboMas_PID_Ctrl_init(&(ctrl_struct->pid_pos));
    RoboMas_PID_Ctrl_init(&(ctrl_struct->pid_vel));
    RoboMas_Actuator_VelocityDob_Reset(&(ctrl_struct->velocity_dob_state));
}

void RoboMas_Init(RoboMas_DeviceInfo dev_info_array[], uint8_t size) {
    for (uint8_t i = 0; i < size; i++) {
        RoboMas_Ctrl_Struct_init(&(dev_info_array[i].ctrl_param));
        dev_info_array[i].ctrl_param.offset_pos = 0.0f;
    }
}

void RoboMas_SendRequest(RoboMas_DeviceInfo dev_info_array[], uint8_t size, float update_freq_hz, CAN_HandleTypeDef *phcan) {
    uint8_t data1[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t data2[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    bool flag_1 = false, flag_2 = false;
    int16_t request_value = 0;
    float diff = 0.0f, t_current = 0.0f, fb_value = 0.0f;

    for (uint8_t i = 0; i < size; i++) {
        const uint8_t device_id = dev_info_array[i].device_id;

        /*
         * Always keep the command frame for every configured CAN group
         * active.  data1/data2 start at zero, so a disabled device gets an
         * explicit zero-current command instead of leaving the C610/C620
         * with its previous current command when it is the last enabled
         * device in the group.
         */
        if (device_id == 0U || device_id > 8U) {
            printf("[RoboMas] device_id must be in 1..8\n\r");
            continue;
        }
        if (device_id < 5U) {
            flag_1 = true;
        } else {
            flag_2 = true;
        }

        /* CANグループは維持し、対象全台に明示的なゼロ電流を送り続ける。 */
        if (ARM_TEST_DISABLE_ROBOMASTER) {
            RoboMas_ControlDisable(&dev_info_array[i]);
            continue;
        }
        const RoboMas_FeedbackData guard_feedback =
            Get_RoboMas_FeedbackData(&dev_info_array[i]);
        /* 受信途絶時に古い速度を使って逆向き電流を出し続けない。 */
        if (device_id == 4U && !RoboMas_Id4FeedbackFresh()) {
            RoboMas_ControlDisable(&dev_info_array[i]);
            id4_diag_active = false;
            continue;
        }
        if (RoboMas_SafetyGuardZero(&dev_info_array[i], &guard_feedback)) {
            continue;
        }

        if (!(dev_info_array[i].ctrl_param._enable_flag) ||
            !(dev_info_array[i].ctrl_param._target_valid)) {
            if (device_id == 4U) {
                id4_diag_active = false;
                id4_velocity_braking = false;
                id4_brake_stopped_cycles = 0U;
            }
            dev_info_array[i].ctrl_param._req_value = 0.0f;
            continue;
        }

        const RoboMas_FeedbackData fb_data =
            Get_RoboMas_FeedbackData(&dev_info_array[i]);

        /* フィードバック確立前は、制御値を計算せずゼロ指令を維持する。 */
        if (fb_data.get_flag == 0U) {
            continue;
        }

        if (dev_info_array[i].ctrl_param._is_calibrating) {
            if (get_switch_state(dev_info_array[i].ctrl_param._limit_port, dev_info_array[i].ctrl_param._limit_pin, dev_info_array[i].ctrl_param._sw_type)) {
                dev_info_array[i].ctrl_param._is_calibrating = false;

                _change_internal_offset_for_calib(&dev_info_array[i]);
                if (device_id == 4U) id4_calibration_completed = true;
                
                RoboMas_ControlDisable(&dev_info_array[i]);
                RoboMas_ChangeControl(&dev_info_array[i], dev_info_array[i].ctrl_param._ctrl_type_before_calib);
                if(robomas_is_position_control(dev_info_array[i].ctrl_param._ctrl_type_before_calib)){
                    RoboMas_SetTarget(&dev_info_array[i], dev_info_array[i].ctrl_param.offset_pos);
                }else{
                    RoboMas_SetTarget(&dev_info_array[i], 0.0f);
                }
                /* ID4は校正終了後に自動で位置保持を開始しない。 */
                if (device_id != 4U) RoboMas_ControlEnable(&dev_info_array[i]);

                // Disable 状態のまま返す
                continue;
            }
        }

        /* 全制御モードで実測速度を監視。ラッチ後は逆向きに減速し、
         * 停止したらDisableする。外向き指令を連送されても再加速しない。 */
        if (device_id == 4U && !id4_endpoint_survey &&
            !dev_info_array[i].ctrl_param._is_calibrating &&
            id4_velocity_stop_required(fb_data.position, fb_data.velocity))
            RoboMas_RequestId4VelocityBrake();
        if (device_id == 4U &&
            dev_info_array[i].ctrl_param.ctrl_type == ROBOMAS_CTRL_POS_MPC) {
            const float error=fabsf(dev_info_array[i].ctrl_param._target_value-fb_data.position);
            if (error>.35f || fabsf(fb_data.velocity)>4.0f) id4_mpc_holding=false;
            if (error<.20f && fabsf(fb_data.velocity)<2.0f) id4_mpc_holding=true;
        } else if (device_id == 4U) id4_mpc_holding=false;
        if (device_id == 4U && id4_velocity_braking && !id4_endpoint_survey &&
            !dev_info_array[i].ctrl_param._is_calibrating) {
            id4_brake_stopped_cycles = fabsf(fb_data.velocity) < 5.0f
                ? id4_brake_stopped_cycles + 1U : 0U;
            if (id4_brake_stopped_cycles >= 20U) {
                robomas_test_guard_tripped = true;
                RoboMas_ControlDisable(&dev_info_array[i]);
                id4_diag_active = false;
                continue;
            }
            /* 高ゲイン通常PIへの切替による微小往復を避け、同じDOBで減速。 */
            t_current = RoboMas_Actuator_VelocityDob_Update(
                &dev_info_array[i].ctrl_param.velocity_dob,
                &dev_info_array[i].ctrl_param.velocity_dob_state,
                0.0f, fb_data.velocity, 1.0f/update_freq_hz);
        } else if (device_id == 4U && id4_mpc_holding &&
                   dev_info_array[i].ctrl_param.ctrl_type == ROBOMAS_CTRL_POS_MPC) {
            /* 到達時の摩擦起因の微小往復を止める。偏差0.35mmで再捕捉する。 */
            t_current=0;
            id4_mpc_reset_requested=true;
            RoboMas_Actuator_VelocityDob_Reset(&dev_info_array[i].ctrl_param.velocity_dob_state);
        } else if (dev_info_array[i].ctrl_param.ctrl_type == ROBOMAS_CTRL_CURRENT) {
            t_current = dev_info_array[i].ctrl_param._target_value;
        } else if (dev_info_array[i].ctrl_param.ctrl_type == ROBOMAS_CTRL_VEL_DOB ||
                   dev_info_array[i].ctrl_param.ctrl_type == ROBOMAS_CTRL_POS_MPC) {
            float velocity_reference = dev_info_array[i].ctrl_param._target_value;
            if (dev_info_array[i].ctrl_param.ctrl_type == ROBOMAS_CTRL_POS_MPC) {
                if (device_id != 4U || !id4_calibration_completed) {
                    RoboMas_ControlDisable(&dev_info_array[i]); continue;
                }
                if (id4_mpc_reset_requested) {
                    Id4PositionMpc_Reset(&id4_mpc); id4_mpc_reset_requested=false;
                }
                const uint32_t begin=HAL_GetTick();
                velocity_reference=Id4PositionMpc_Update(&id4_mpc,
                    fb_data.position, fb_data.velocity, velocity_reference,
                    dev_info_array[i].ctrl_param.velocity_dob.reference_alpha,
                    1.0f/update_freq_hz);
                const uint32_t elapsed=HAL_GetTick()-begin;
                if (elapsed>id4_mpc_max_runtime_ms) id4_mpc_max_runtime_ms=elapsed;
            }
            if (device_id == 4U) {
                /* Mode4の従来800mm/s制限と、Mode5の無負荷仕様上限は分離する。 */
                if (dev_info_array[i].ctrl_param.ctrl_type == ROBOMAS_CTRL_VEL_DOB)
                    velocity_reference = id4_velocity_reference_limit(velocity_reference);
                velocity_reference = id4_safe_velocity(fb_data.position, velocity_reference);
                if (robomas_test_guard_armed &&
                    id4_velocity_stop_required(fb_data.position, fb_data.velocity))
                    RoboMas_RequestId4VelocityBrake();
                if (id4_velocity_braking) {
                    velocity_reference = 0.0f;
                    id4_brake_stopped_cycles = fabsf(fb_data.velocity) < 5.0f
                        ? id4_brake_stopped_cycles + 1U : 0U;
                    if (id4_brake_stopped_cycles >= 20U) {
                        RoboMas_ControlDisable(&dev_info_array[i]);
                        id4_diag_active = false;
                        continue;
                    }
                }
            }
            if (device_id == 4U && !id4_diag_active) {
                id4_diag_active = true;
                id4_diag_count = 0U;
                id4_diag_read = 0U;
                id4_diag_started = false;
            }
            const float dt = update_freq_hz > 0.0f
                                 ? 1.0f / update_freq_hz
                                 : 0.0f;
            t_current = RoboMas_Actuator_VelocityDob_Update(
                &(dev_info_array[i].ctrl_param.velocity_dob),
                &(dev_info_array[i].ctrl_param.velocity_dob_state),
                velocity_reference,
                fb_data.velocity,
                dt);
            if (device_id == 4U) {
                if (fabsf(dev_info_array[i].ctrl_param._target_value) > 0.1f)
                    id4_diag_started = true;
                if (id4_diag_started && id4_diag_count < 512U) {
                    const RoboMas_Actuator_VelocityDob_State *s =
                        &dev_info_array[i].ctrl_param.velocity_dob_state;
                    Id4Diagnostic *d = &id4_diag[id4_diag_count];
                    d->tick = HAL_GetTick();
                    d->position = fb_data.position;
                    d->velocity = fb_data.velocity;
                    d->target = velocity_reference;
                    d->model = s->omega_model;
                    d->current = t_current;
                    d->unsaturated = s->unsaturated_torque /
                        dev_info_array[i].ctrl_param.velocity_dob.K_tau;
                    d->saturated = s->output_saturated ? 1U : 0U;
                    ++id4_diag_count;
                }
            }
        } else {
            switch (dev_info_array[i].ctrl_param.ctrl_type) {
                case ROBOMAS_CTRL_POS:
                case ROBOMAS_CTRL_POS_AW:
                    fb_value = fb_data.position;
                    break;
                case ROBOMAS_CTRL_VEL:
                    fb_value = fb_data.velocity;
                    break;
                default:
                    fb_value = 0.0f;
                    break;
            }
            diff = dev_info_array[i].ctrl_param._target_value - fb_value;
            if(robomas_is_position_control(dev_info_array[i].ctrl_param.ctrl_type)) {
                float t_vel = RoboMas_PID_Ctrl_AW(&(dev_info_array[i].ctrl_param.pid_pos), diff, dev_info_array[i].ctrl_param.velocity_limit == ROBOMAS_LIMIT_ENABLE, dev_info_array[i].ctrl_param.velocity_limit_size, update_freq_hz);
                if (device_id == 4U && !id4_endpoint_survey)
                    t_vel = id4_safe_velocity(fb_data.position, t_vel);
                t_current = RoboMas_PID_Ctrl_AW(&(dev_info_array[i].ctrl_param.pid_vel), t_vel - fb_data.velocity, dev_info_array[i].ctrl_param.current_limit == ROBOMAS_LIMIT_ENABLE, dev_info_array[i].ctrl_param.current_limit_size, update_freq_hz);
                // 位置制御の場合は速度と位置の2重でPID
            }else if(dev_info_array[i].ctrl_param.ctrl_type == ROBOMAS_CTRL_VEL){
                if (device_id == 4U && !id4_endpoint_survey &&
                    !dev_info_array[i].ctrl_param._is_calibrating)
                    diff = id4_safe_velocity(fb_data.position,
                        dev_info_array[i].ctrl_param._target_value) - fb_data.velocity;
                const bool id4_calibrating = device_id == 4U &&
                    dev_info_array[i].ctrl_param._is_calibrating;
                t_current = RoboMas_PID_Ctrl_AW(&(dev_info_array[i].ctrl_param.pid_vel), diff,
                    id4_calibrating || dev_info_array[i].ctrl_param.current_limit == ROBOMAS_LIMIT_ENABLE,
                    id4_calibrating ? ID4_CALIBRATION_CURRENT_A :
                        dev_info_array[i].ctrl_param.current_limit_size, update_freq_hz);
            }
        }
        /*
         * Disable may arrive from the micro-ROS task while the controller
         * calculation above is in progress.  Discard that calculation if
         * the device was disabled before packing the CAN frame.
         */
        if (!(dev_info_array[i].ctrl_param._enable_flag) ||
            !(dev_info_array[i].ctrl_param._target_valid)) {
            dev_info_array[i].ctrl_param._req_value = 0.0f;
            continue;
        }

        /* 校正2A、通常制御はID4_NORMAL_CURRENT_AをCAN送信直前にも強制する。 */
        if (device_id == 4U)
            t_current = _clip_f_abs(t_current,
                dev_info_array[i].ctrl_param._is_calibrating
                    ? ID4_CALIBRATION_CURRENT_A : ID4_NORMAL_CURRENT_A);
        // 目標値の計算
        switch (dev_info_array[i].device_type) {
            case ROBOMASTER_C610:
                dev_info_array[i].ctrl_param._req_value = _clip_f_abs(t_current, 10.0f);
                request_value = c610_current_f2int( dev_info_array[i].ctrl_param._req_value);
                break;
            case ROBOMASTER_C620:
                dev_info_array[i].ctrl_param._req_value = _clip_f_abs(t_current, 20.0f);
                request_value = c620_current_f2int( dev_info_array[i].ctrl_param._req_value);
                break;
            default:
                request_value = 2;
                break;
        }
//        printf("request_val1:%d",request_value);
        if(dev_info_array[i].ctrl_param.rotation == ROBOMAS_ROT_CW){
            request_value *= -1;
        }

        // 各モーターの目標値の設定
        if (device_id < 5U) {
            for (uint8_t j = 0; j < 2; j++) {
                data1[(device_id - 1U) * 2U + j] = (request_value >> ((!j) * 8)) & 0b11111111;
            }
        } else {
            for (uint8_t j = 0; j < 2; j++) {
                data2[(device_id - 5U) * 2U + j] = (request_value >> ((!j) * 8)) & 0b11111111;
            }
        }
    }
    if (flag_1)RoboMas_SendBytes(phcan, 0x200, (uint8_t *) data1, sizeof(data1));
    if (flag_2)RoboMas_SendBytes(phcan, 0x1FF, (uint8_t *) data2, sizeof(data2));
}

void RoboMas_WaitForConnect(RoboMas_DeviceInfo dev_info_array[], uint8_t size, DelayFunction_t f_delay) {
    bool all_connected;

    printf("[RoboMas] Wait for Connection...\r\n");
    do {
        all_connected = true;
        for (uint8_t i = 0; i < size; i++) {
            const RoboMas_FeedbackData feedback = Get_RoboMas_FeedbackData(&dev_info_array[i]);
            if (feedback.get_flag == 0U) {
                all_connected = false;
                break;
            }
        }
        if (!all_connected) {
            f_delay(5U);
        }
    } while (!all_connected);

    printf("[RoboMas] All Connected!\r\n");
}


static bool get_switch_state(GPIO_TypeDef* limit_port, uint16_t limit_pin, ROBOMAS_SWITCH_TYPE sw_type){
    if(sw_type == ROBOMAS_SWITCH_NO){
        return HAL_GPIO_ReadPin(limit_port, limit_pin);
    }else{
        return !HAL_GPIO_ReadPin(limit_port, limit_pin);
    }
}

void RoboMas_send_current(RoboMas_DeviceInfo *device_info, float current, CAN_HandleTypeDef *phcan){
    int16_t request_value;
    switch (device_info->device_type) {
        case ROBOMASTER_C610:
            request_value = c610_current_f2int(current);
            break;
        case ROBOMASTER_C620:
            request_value = c620_current_f2int(current);
            break;
        default:
            request_value = 0;
            break;
    }
    // printf("request_val2:%d",request_value);
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    if(device_info->device_id < 5) {
        for (uint8_t j = 0; j < 2; j++) {
            data[(device_info->device_id - 1) * 2 + j] = (request_value >> ((!j) * 8)) & 0b11111111;
        }
        RoboMas_SendBytes(phcan, 0x200, (uint8_t *) data, sizeof(data));
    }else{
        for (uint8_t j = 0; j < 2; j++) {
            data[(device_info->device_id - 5) * 2 + j] = (request_value >> ((!j) * 8)) & 0b11111111;
        }
        RoboMas_SendBytes(phcan, 0x1FF, (uint8_t *) data, sizeof(data));
    }
}

void RoboMas_Calibration(RoboMas_DeviceInfo *device_info, float calib_vel, ROBOMAS_SWITCH_TYPE sw_type, GPIO_TypeDef* limit_port, uint16_t limit_pin, CAN_HandleTypeDef *phcan){
    if (ARM_TEST_DISABLE_ROBOMASTER) return;
    if(device_info->ctrl_param.use_internal_offset != ROBOMAS_USE_OFFSET_POS_CALIB) return;

    device_info->ctrl_param._sw_type = sw_type;
    device_info->ctrl_param._limit_port = limit_port;
    device_info->ctrl_param._limit_pin = limit_pin;
    device_info->ctrl_param._ctrl_type_before_calib = device_info->ctrl_param.ctrl_type;

    RoboMas_ControlDisable(device_info);
    RoboMas_ChangeControl(device_info, ROBOMAS_CTRL_VEL);
    RoboMas_SetTarget(device_info, calib_vel);
    device_info->ctrl_param._is_calibrating = true;
    RoboMas_ControlEnable(device_info);
}

void RoboMas_ChangeControl(RoboMas_DeviceInfo *dev_info, ROBOMAS_CTRL_TYPE new_ctrl_type) {
    if (dev_info->device_id == 4U) id4_mpc_reset_requested=true;
    RoboMas_Ctrl_Struct_init(&(dev_info->ctrl_param));
    dev_info->ctrl_param.ctrl_type = new_ctrl_type;
}

void RoboMas_SetTarget(RoboMas_DeviceInfo *device_info, float target_value) {
    if(!isfinite(target_value)) return;
    if(device_info->ctrl_param._is_calibrating) return;
    
    device_info->ctrl_param._target_value = target_value;
    device_info->ctrl_param._target_valid = true;
}

void RoboMas_ControlEnable(RoboMas_DeviceInfo *dev_info) {
    if (ARM_TEST_DISABLE_ROBOMASTER) return;
    if (dev_info->device_id == ROBOMAS_TEST_GUARD_ID &&
        robomas_test_guard_tripped) return;
    dev_info->ctrl_param._enable_flag = true;
}

void RoboMas_ControlDisable(RoboMas_DeviceInfo *dev_info) {
    if (dev_info->device_id == 4U) id4_mpc_reset_requested=true;
    /*
     * Disabling must be a safe state transition, not only a scheduling
     * switch.  RoboMas_SendRequest() sends the zero-current CAN frame on its
     * next cycle; clearing the target and PID state prevents the previous
     * velocity command/integrator from being resumed on the next Enable.
     */
    /* Invalidate the command before the task can observe a new enable. */
    dev_info->ctrl_param._target_valid = false;
    dev_info->ctrl_param._enable_flag = false;
    dev_info->ctrl_param._target_value = 0.0f;
    dev_info->ctrl_param._req_value = 0.0f;
    RoboMas_PID_Ctrl_init(&(dev_info->ctrl_param.pid_pos));
    RoboMas_PID_Ctrl_init(&(dev_info->ctrl_param.pid_vel));
    RoboMas_Actuator_VelocityDob_Reset(&(dev_info->ctrl_param.velocity_dob_state));
}

bool RoboMas_IsCalibrationEnded(RoboMas_DeviceInfo *dev_info){
    return !(dev_info->ctrl_param._is_calibrating);
}

void RoboMas_Wait_For_Calib(RoboMas_DeviceInfo *dev_info, DelayFunction_t f_delay){
    while(dev_info->ctrl_param._is_calibrating) f_delay(5);
}

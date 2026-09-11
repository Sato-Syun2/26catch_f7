// Includes --------------------------------

#include <main.h> // HALライブラリの定義が含まれているヘッダ

#include <CAN_Robstride.h>
#include <CAN_Robstride_Def.h>
#include <CAN_Robstride_System.h>
#include <robstride_constant.h>
#include <Robstride_Control.h>
#include "arm_test_config.h"
#include "arm_feedback_guard.h"
#include "Control/ArmVelocityEstimate.h"
#include "Control/ArmPositionMpc.h"

#include <stdbool.h>
#include <math.h>  // 数学関数 (fabsf, fmaxf, fminf など) を使用するためにインクルード
#include <stdio.h> // 標準入出力関数 (printf など) を使用するためにインクルード

#define ROBSTRIDE_SERVICE_TIMEOUT_MS       (300U)
#define ROBSTRIDE_SERVICE_RESPONSE_WINDOW_MS (10U)
#define ROBSTRIDE_PARAMETER_TOLERANCE     (0.0005f)
#define ROBSTRIDE_POSITION_TURN_RAD       (6.28318530717958647692f)
#define ROBSTRIDE_POSITION_HALF_TURN_RAD  (3.14159265358979323846f)
#define ROBSTRIDE_POSITION_STEP_TOLERANCE_RAD (0.02f)

static bool arm_guard_tripped[3];
static bool arm_current_config_ok[3];
static ArmVelocityEstimate arm_velocity_estimate[3];
static ArmPositionMpc arm_position_mpc[3];
/* 同定用RAM記録。CAN追加読み出しなし、各軸50Hz、単軸約82秒。 */
typedef struct {
    uint32_t tick, id;
    float target, model, velocity, current, position, reference_current;
} ArmDobSample;
volatile struct { uint32_t count; ArmDobSample samples[4096]; } arm_dob_log;
static uint32_t arm_dob_log_tick[3];
/* 肘の高周期診断。CAN追加読み取りなし。最後の4.096秒を保持。 */
typedef struct {
    uint32_t tick, feedback_tick;
    float position, raw_velocity, filtered_velocity, command_current, reference_current, target;
} ArmFastSample;
volatile struct { uint32_t count; ArmFastSample samples[2048]; } arm_fast_log;
static bool arm_fast_capture_started;
/* 起動後の非動作試験で全リングを連続書き込み。最初のDOB開始後は停止する。
 * 実測区間ではこの疑似データは使わず、停止後の実測リングを保存する。 */
void Robstride_FastLogIdleProbe(void)
{
    if (arm_fast_capture_started) return;
    const uint32_t count=arm_fast_log.count;
    const uint32_t now=HAL_GetTick();
    arm_fast_log.samples[count%2048U]=(ArmFastSample){.tick=now,.feedback_tick=now,.target=NAN};
    __DMB();
    arm_fast_log.count=count+1U;
}
bool Robstride_ArmGuardCheck(Robstride_DeviceInfo *device)
{
    if (!device) return false;
    const uint8_t id = device->device_id;
    if (id != 1U && id != 2U) return true;
    float position = 0.0f;
    uint32_t tick = 0U;
    const bool measured = Robstride_ReadMeasuredPosition(device, &position, &tick);
    return ArmFeedbackGuard_Check(measured, (uint32_t)(HAL_GetTick() - tick),
        measured && ArmPositionMpc_TargetAllowedForDevice(id, position),
        device->ctrl_param._enable_flag != 0U, &arm_guard_tripped[id]);
}

// Private Function Prototypes --------------------------------


static void Robstride_PID_Ctrl_init(Robstride_PID_StructTypedef *const params);
static void Robstride_Ctrl_Struct_init(Robstride_Ctrl_StructTypedef *ctrl_struct);
// static void Check_CAN_Error(CAN_HandleTypeDef *phcan);
// static void Print_CAN_BitTiming_Params(CAN_HandleTypeDef *hcan);
static uint8_t Robstride_get_switch_state(GPIO_TypeDef *limit_port, uint32_t limit_pin, ROBSTRIDE_SWITCH_TYPE sw_type);
static void Robstride_SetMechposToZero(Robstride_DeviceInfo *dev_info, DelayFunction_t f_delay);
static bool robstride_deadline_reached(uint32_t start_tick, uint32_t timeout_ms);
static bool robstride_wait_for_feedback(Robstride_DeviceInfo *dev_info,
                                        uint32_t sequence_before,
                                        uint32_t start_tick,
                                        DelayFunction_t f_delay);
static bool robstride_wait_for_parameter(Robstride_DeviceInfo *dev_info,
                                         uint16_t address,
                                         uint32_t sequence_before,
                                         uint32_t start_tick,
                                         DelayFunction_t f_delay);
static bool robstride_position_target_to_wire(
    const Robstride_DeviceInfo *device_info,
    float target_value,
    float *wire_value);
static bool robstride_feedback_position_out_of_range(
    const Robstride_DeviceInfo *device_info);
static void robstride_trip_position_guard(Robstride_DeviceInfo *device_info);
static bool robstride_target_parameter(const Robstride_DeviceInfo *device_info,
                                       float target_value,
                                       uint16_t *address,
                                       float *wire_value);
static float robstride_clamp_current(const Robstride_DeviceInfo *device_info,
                                     float current);
static bool robstride_write_int_verified(Robstride_DeviceInfo *device_info,
                                         uint16_t address,
                                         int value,
                                         uint8_t expected_value,
                                         DelayFunction_t f_delay);
static bool robstride_set_target_verified(Robstride_DeviceInfo *device_info,
                                          float target_value,
                                          DelayFunction_t f_delay);
static HAL_StatusTypeDef robstride_send_current(Robstride_DeviceInfo *device_info,
                                                float current);
static HAL_StatusTypeDef robstride_set_target_internal(
    Robstride_DeviceInfo *device_info,
    float target_value,
    bool priority,
    bool generation_guarded,
    uint32_t expected_generation);
static bool robstride_control_command_verified(Robstride_DeviceInfo *device_info,
                                               uint8_t command_id,
                                               uint8_t expected_state,
                                                DelayFunction_t f_delay);
static uint8_t robstride_wire_control_type(ROBSTRIDE_CTRL_TYPE ctrl_type);

// Functions --------------------------------

/**
 * @brief Robstride PID制御パラメータ構造体を初期化します。
 * @param params 初期化対象のRobstride_PID_StructTypedef構造体へのポインタ
 * @retval なし
 */
static void Robstride_PID_Ctrl_init(Robstride_PID_StructTypedef *const params) {
    params->kp_pos = 0.5f;      // 位置制御Pゲインのデフォルト値
    params->kp_vel = 10.0f;     // 速度制御Pゲインのデフォルト値
    params->ki_vel = 5.0f;      // 速度制御Iゲインのデフォルト値
    params->filter_vel = 1.0f;  // 速度制御フィルタゲインのデフォルト値
    params->kp_cur = 0.05f;     // 電流制御Pゲインのデフォルト値
    params->ki_cur = 0.05f;     // 電流制御Iゲインのデフォルト値
    params->filter_cur = 0.06f; // 電流制御フィルタゲインのデフォルト値
    params->_integral = 0.0f;   // 積分項の初期値
    params->_prev_value = 0.0f; // 前回値の初期値
}

/**
 * @brief Robstride制御構造体を初期化します。
 * @param ctrl_struct 初期化対象のRobstride_Ctrl_StructTypedef構造体へのポインタ
 * @retval なし
 */
static void Robstride_Ctrl_Struct_init(Robstride_Ctrl_StructTypedef *const ctrl_struct) {
    Robstride_Actuator_VelocityDob_Reset(&(ctrl_struct->velocity_dob_state));
    ctrl_struct->_req_value = 0.0f;
    ctrl_struct->_mode_configured = 0U;
    ctrl_struct->_target_value = 0.0f;            // 目標値の初期値
    ctrl_struct->_enable_flag = 0;                // 有効フラグの初期値 (無効)
    ctrl_struct->_position_guard_latched = 0U;
    ctrl_struct->_target_generation = 0U;
    ctrl_struct->_position_target_wire = 0.0f;
    ctrl_struct->_position_target_valid = 0U;
    Robstride_PID_Ctrl_init(&(ctrl_struct->pid)); // PIDパラメータ構造体を初期化
}

/**
 * @brief Robstrideデバイス情報配列を初期化します。
 * @param dev_info_array Robstrideデバイス情報構造体の配列
 * @param size 配列のサイズ
 * @retval なし
 */
void Robstride_Init(Robstride_DeviceInfo dev_info_array[], const uint8_t size) {
    for (uint8_t i = 0; i < size; i++) {
        Robstride_Ctrl_Struct_init(&(dev_info_array[i].ctrl_param)); // 各デバイスの制御構造体を初期化
    }
}

static uint8_t robstride_wire_control_type(const ROBSTRIDE_CTRL_TYPE ctrl_type)
{
    /* VEL_DOBはF7側制御なので、モータ内部は電流モードで動かす。 */
    return (ctrl_type == ROBSTRIDE_CTRL_VEL_DOB || ctrl_type == ROBSTRIDE_CTRL_POS_MPC)
               ? (uint8_t)ROBSTRIDE_CTRL_CURRENT
               : (uint8_t)ctrl_type;
}

static bool robstride_deadline_reached(const uint32_t start_tick,
                                       const uint32_t timeout_ms)
{
    return ((uint32_t)(HAL_GetTick() - start_tick) >= timeout_ms);
}

static bool robstride_wait_for_feedback(Robstride_DeviceInfo *const dev_info,
                                        const uint32_t sequence_before,
                                        const uint32_t start_tick,
                                        DelayFunction_t f_delay)
{
    while (!robstride_deadline_reached(start_tick,
                                       ROBSTRIDE_SERVICE_RESPONSE_WINDOW_MS)) {
        f_delay(1U);
        if (Robstride_GetFeedbackSequence(dev_info) != sequence_before) {
            return true;
        }
    }
    return Robstride_GetFeedbackSequence(dev_info) != sequence_before;
}

static bool robstride_wait_for_parameter(Robstride_DeviceInfo *const dev_info,
                                         const uint16_t address,
                                         const uint32_t sequence_before,
                                         const uint32_t start_tick,
                                         DelayFunction_t f_delay)
{
    while (!robstride_deadline_reached(start_tick,
                                       ROBSTRIDE_SERVICE_RESPONSE_WINDOW_MS)) {
        f_delay(1U);
        if (Robstride_GetParameterSequence(dev_info, address) != sequence_before) {
            return true;
        }
    }
    return Robstride_GetParameterSequence(dev_info, address) != sequence_before;
}

/*
 * 位置指令をモータ座標へ変換する。
 *
 * Robstrideの位置パラメータは有限範囲なので、ROS側の値を無条件に
 * modulo変換すると、境界で別周回へ飛ぶ。まず入力が表現可能か確認し、
 * フィードバックが既に周回を跨いでいる場合だけ同値な候補を選ぶ。
 * 候補がモータの位置範囲に戻せない場合は、危険な指令として拒否する。
 */
static bool robstride_position_target_to_wire(
    const Robstride_DeviceInfo *const device_info,
    const float target_value,
    float *const wire_value)
{
    float min_position;
    float max_position;
    if (!Robstride_GetPositionLimits(device_info->device,
                                     &min_position,
                                     &max_position) ||
        !isfinite(target_value) ||
        !isfinite(device_info->ctrl_param.offset_pos) ||
        !isfinite(device_info->ctrl_param.quant_per_rot) ||
        device_info->ctrl_param.quant_per_rot <= 0.0f) {
        return false;
    }

    float target_position = target_value - device_info->ctrl_param.offset_pos;
    target_position /= device_info->ctrl_param.quant_per_rot;
    if (device_info->ctrl_param.rotation == ROBSTRIDE_ROT_CW) {
        target_position *= -1.0f;
    }

    if (!isfinite(target_position)) {
        return false;
    }

    const Robstride_FeedbackData feedback =
        Read_Robstride_FeedbackData((Robstride_DeviceInfo *)device_info);
    if ((feedback.get_flag != 0U) && isfinite(feedback.position)) {
        float current_position = feedback.position -
                                 device_info->ctrl_param.offset_pos;
        current_position /= device_info->ctrl_param.quant_per_rot;
        if (device_info->ctrl_param.rotation == ROBSTRIDE_ROT_CW) {
            current_position *= -1.0f;
        }

        if (!isfinite(current_position) ||
            !isfinite(ROBSTRIDE_POSITION_TURN_RAD) ||
            ROBSTRIDE_POSITION_TURN_RAD <= 0.0f) {
            return false;
        }

        const float reference_position =
            (device_info->ctrl_param._position_target_valid != 0U)
                ? device_info->ctrl_param._position_target_wire
                : current_position;
        if (!isfinite(reference_position)) {
            return false;
        }

        /*
         * ROSの角度は出力軸の度数法であり、1回転した角度は同じ姿勢を
         * 表す。通信値の±4π幅を周期にすると、±180°付近の指令変化を
         * 数百度の逆回転として解釈するため、ここでは1回転周期で
         * 現在位置に最も近い同値目標を選ぶ。
        */
        const float turn_offset =
            roundf((reference_position - target_position) /
                   ROBSTRIDE_POSITION_TURN_RAD);
        const float candidate = target_position +
                                turn_offset * ROBSTRIDE_POSITION_TURN_RAD;
        const float target_step = fabsf(candidate - reference_position);
        const float feedback_step = fabsf(candidate - current_position);
        if (!isfinite(candidate) ||
            !isfinite(target_step) ||
            !isfinite(feedback_step) ||
            target_step > (ROBSTRIDE_POSITION_HALF_TURN_RAD +
                           ROBSTRIDE_POSITION_STEP_TOLERANCE_RAD) ||
            feedback_step > (ROBSTRIDE_POSITION_HALF_TURN_RAD +
                             ROBSTRIDE_POSITION_STEP_TOLERANCE_RAD) ||
            candidate < min_position ||
            candidate > max_position) {
            return false;
        }
        target_position = candidate;
    } else if (target_position < min_position ||
               target_position > max_position) {
        /* フィードバックなしで別周回を選ぶ根拠がないため拒否する。 */
        return false;
    }

    *wire_value = target_position;
    return true;
}

static bool robstride_feedback_position_out_of_range(
    const Robstride_DeviceInfo *const device_info)
{
    float min_position;
    float max_position;
    if (!Robstride_GetPositionLimits(device_info->device,
                                     &min_position,
                                     &max_position) ||
        !isfinite(device_info->ctrl_param.offset_pos) ||
        !isfinite(device_info->ctrl_param.quant_per_rot) ||
        device_info->ctrl_param.quant_per_rot <= 0.0f) {
        return false;
    }

    const Robstride_FeedbackData feedback =
        Read_Robstride_FeedbackData((Robstride_DeviceInfo *)device_info);
    if ((feedback.get_flag == 0U) || !isfinite(feedback.position)) {
        return false;
    }

    float position = feedback.position - device_info->ctrl_param.offset_pos;
    position /= device_info->ctrl_param.quant_per_rot;
    if (device_info->ctrl_param.rotation == ROBSTRIDE_ROT_CW) {
        position *= -1.0f;
    }
    return !isfinite(position) ||
           position < min_position ||
           position > max_position;
}

/* 位置指令を安全に表現できない場合は、次の目標値を送らず即時にRESETする。 */
static void robstride_trip_position_guard(
    Robstride_DeviceInfo *const device_info)
{
    if ((device_info->ctrl_param.ctrl_type != ROBSTRIDE_CTRL_POS) ||
        (device_info->ctrl_param._position_guard_latched != 0U)) {
        return;
    }

    const uint8_t data[8] = {0U};
    /*
     * 既に通常キューやCANメールボックスへ入った位置指令を残したまま
     * RESETを送ると、RESET後に古い指令が出て再び回転する可能性がある。
     * 通常送信を停止し、メールボックスも中断してからRESETを送る。
     */
    Robstride_BeginPriorityTransaction(device_info->phcan);
    Robstride_ClearPriorityTxQueue(device_info->phcan);
    (void)Robstride_SendPriorityBytes(device_info->phcan,
                                      device_info->device_id,
                                      CMD_RESET,
                                      device_info->master_id,
                                      data,
                                      sizeof(data));
    Robstride_EndPriorityTransaction(device_info->phcan);
    device_info->ctrl_param._position_guard_latched = 1U;
    device_info->ctrl_param._enable_flag = 0U;
    device_info->ctrl_param._position_target_valid = 0U;
    Robstride_InvalidateTargetGeneration(device_info);
}

static bool robstride_target_parameter(const Robstride_DeviceInfo *const device_info,
                                       const float target_value,
                                       uint16_t *const address,
                                       float *const wire_value)
{
    if (!isfinite(target_value) ||
        !isfinite(device_info->ctrl_param.quant_per_rot) ||
        device_info->ctrl_param.quant_per_rot <= 0.0f) {
        return false;
    }

    float value = target_value;
    switch (device_info->ctrl_param.ctrl_type) {
        case ROBSTRIDE_CTRL_POS:
            if ((device_info->device_id == 1U || device_info->device_id == 2U) &&
                !ArmPositionMpc_TargetAllowedForDevice(device_info->device_id, value)) {
                arm_guard_tripped[device_info->device_id] = true;
                return false;
            }
            *address = (uint16_t)ADDR_LOC_REF;
            if (robstride_feedback_position_out_of_range(device_info)) {
                return false;
            }
            if (!robstride_position_target_to_wire(device_info,
                                                   target_value,
                                                   wire_value)) {
                return false;
            }
            value = *wire_value;
            break;
        case ROBSTRIDE_CTRL_VEL:
            *address = (uint16_t)ADDR_SPEED_REF;
            value /= device_info->ctrl_param.quant_per_rot;
            if (device_info->ctrl_param.rotation == ROBSTRIDE_ROT_CW) {
                value *= -1.0f;
            }
            break;
        case ROBSTRIDE_CTRL_CURRENT:
            *address = (uint16_t)ADDR_IQ_REF;
            value = robstride_clamp_current(device_info, value);
            if (device_info->ctrl_param.rotation == ROBSTRIDE_ROT_CW) {
                value *= -1.0f;
            }
            break;
        default:
            return false;
    }

    *wire_value = value;
    return isfinite(value);
}

static float robstride_clamp_current(const Robstride_DeviceInfo *const device_info,
                                     const float current)
{
    if (device_info == NULL || !isfinite(current)) {
        return 0.0f;
    }

    const float limit = fabsf(device_info->ctrl_param.current_limit_size);
    if (!isfinite(limit) || limit <= 0.0f) {
        return 0.0f;
    }

    return fmaxf(-limit, fminf(current, limit));
}

static float robstride_cached_parameter(const Robstride_FeedbackData *const feedback,
                                        const uint16_t address)
{
    switch (address) {
        case ADDR_IQ_REF:
            return feedback->iq_ref;
        case ADDR_SPEED_REF:
            return feedback->spd_ref;
        case ADDR_LOC_REF:
            return feedback->loc_ref;
        case ADDR_LIMIT_SPEED:
            return feedback->limit_spd;
        case ADDR_LIMIT_CURRENT:
            return feedback->limit_cur;
        case ADDR_CURRENT_KP: return feedback->cur_kp;
        case ADDR_CURRENT_KI: return feedback->cur_ki;
        case ADDR_CURRENT_FILTER_GAIN: return feedback->cur_filt_gain;
        default:
            return NAN;
    }
}

static bool robstride_write_int_verified(Robstride_DeviceInfo *const device_info,
                                         const uint16_t address,
                                         const int value,
                                         const uint8_t expected_value,
                                         DelayFunction_t f_delay)
{
    bool success = false;
    const uint32_t transaction_start = HAL_GetTick();
    uint8_t expected_data = expected_value;

    Robstride_BeginPriorityTransaction(device_info->phcan);
    while (!robstride_deadline_reached(transaction_start,
                                       ROBSTRIDE_SERVICE_TIMEOUT_MS)) {
        Robstride_ClearPriorityTxQueue(device_info->phcan);

        const uint32_t feedback_sequence =
            Robstride_GetFeedbackSequence(device_info);
        if (Robstride_WriteIntDataPriority(device_info, address, value) == HAL_OK &&
            robstride_wait_for_feedback(device_info,
                                        feedback_sequence,
                                        HAL_GetTick(),
                                        f_delay)) {
            const uint32_t parameter_sequence =
                Robstride_GetParameterSequence(device_info, address);
            if (Robstride_RequestReadParameterPriority(device_info, address) == HAL_OK &&
                robstride_wait_for_parameter(device_info,
                                             address,
                                             parameter_sequence,
                                             HAL_GetTick(),
                                             f_delay)) {
                const Robstride_FeedbackData feedback =
                    Read_Robstride_FeedbackData(device_info);
                if (address == ADDR_RUN_MODE &&
                    feedback.run_mode == expected_data) {
                    success = true;
                    break;
                }
            }
        }
        f_delay(1U);
    }

    Robstride_ClearPriorityTxQueue(device_info->phcan);
    Robstride_EndPriorityTransaction(device_info->phcan);
    return success;
}

static bool robstride_control_command_verified(Robstride_DeviceInfo *const device_info,
                                                const uint8_t command_id,
                                                const uint8_t expected_state,
                                                DelayFunction_t f_delay)
{
    bool success = false;
    const uint32_t transaction_start = HAL_GetTick();
    const uint8_t data[8] = {0U};

    Robstride_BeginPriorityTransaction(device_info->phcan);
    while (!robstride_deadline_reached(transaction_start,
                                       ROBSTRIDE_SERVICE_TIMEOUT_MS)) {
        Robstride_ClearPriorityTxQueue(device_info->phcan);

        const uint32_t feedback_sequence =
            Robstride_GetFeedbackSequence(device_info);
        if (Robstride_SendPriorityBytes(device_info->phcan,
                                        device_info->device_id,
                                        command_id,
                                        device_info->master_id,
                                        data,
                                        sizeof(data)) == HAL_OK &&
            robstride_wait_for_feedback(device_info,
                                        feedback_sequence,
                                        HAL_GetTick(),
                                        f_delay)) {
            const Robstride_FeedbackData feedback =
                Read_Robstride_FeedbackData(device_info);
            if (feedback.mode_status == expected_state) {
                /* Type 2のfeedbackは、送信直前のsequenceを基準にして
                 * 送信後に受信したことを確認済みである。Enable/Disableの
                 * 成否はmode_statusで確定し、別途Type 17のrun_mode読み出し
                 * まで要求しない。Type 17応答を返さない個体でも、今回の
                 * 制御命令そのものが成功している場合があるためである。 */
                success = true;
                break;
            }
        }
        f_delay(1U);
    }

    Robstride_ClearPriorityTxQueue(device_info->phcan);
    Robstride_EndPriorityTransaction(device_info->phcan);
    return success;
}

void Robstride_WaitForConnect(Robstride_DeviceInfo dev_info_array[], const uint8_t size, DelayFunction_t f_delay) {
    uint8_t flag = 0;
    printf("[Robstride] Wait for Connection...\r\n");
    while (!flag) {
        flag = 1;
        for (uint8_t i = 0; i < size; i++) {
            /*
             * 電源投入前は応答が来ないため、応答待ちの Robstride_ControlDisable()
             * をここで使うと以後の再送が止まる。リセット要求を1回だけ送信し、
             * 次の周回で再試行する。
             */
            const uint8_t data[8] = {0};
            Robstride_SendBytes(dev_info_array[i].phcan,
                                dev_info_array[i].device_id,
                                CMD_RESET,
                                dev_info_array[i].master_id,
                                data,
                                sizeof(data));
            // printf("Checking Active Report Status for motor %d...\n\r", i + 1);
            // Robstride_CheckActiveReportStatus(&dev_info_array[i]);
            f_delay(50); // 応答を待つためのディレイ
            // Check_CAN_Error(dev_info_array[i].phcan); // CANエラーをチェック
            // Print_CAN_BitTiming_Params(dev_info_array[i].phcan); // CANビットタイミングパラメータを表示
            if (!Read_Robstride_FeedbackData(&dev_info_array[i]).get_flag) {
                printf("[Robstride] Device id : %d Not Connected...\r\n", dev_info_array[i].device_id);
                // 1台でも接続されていなければ (get_flagが0なら)、フラグを0にしてループを継続
                flag = 0;
            } else {
                printf("[Robstride] Device id : %d Connected!\r\n", dev_info_array[i].device_id);
            }
        }
        f_delay(100);
    }

    printf("[Robstride] All Connected!\r\n");
    f_delay(500);
}

#if 0 // 使われていないので一旦コメントアウト
/**
 * @brief CANのエラーステータスを確認し、内容をシリアルコンソールに出力します。
 * @param hcan 確認対象のCANハンドル
 */
static void Check_CAN_Error(CAN_HandleTypeDef *const phcan) {
    // HAL_CAN_GetError() を呼び出して現在のエラーコードを取得
    const uint32_t can_error = HAL_CAN_GetError(phcan);

    // エラーが存在するかどうかを確認
    if (can_error == HAL_CAN_ERROR_NONE) {
        // エラーなし
        // printf("CAN No Error.\r\n"); // 正常時に毎回表示するとログが煩雑になるため、コメントアウトを推奨
        return; // エラーがなければここで処理終了
    }

    // --- 何らかのエラーが検出された場合、以下で詳細を判別 ---

    printf("--- CAN Error Detected! Code: 0x%08lX ---\r\n", can_error);

    // === バス状態とプロトコルに関するエラー ===
    if ((can_error & HAL_CAN_ERROR_BOF) != 0) {
        printf("  - Bus-off error: バスから切り離されました。重大なエラーです。\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_EPV) != 0) {
        printf("  - Error Passive: エラーパッシブ状態です (RECまたはTEC > 127)。\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_EWG) != 0) {
        printf("  - Protocol Error Warning: エラーワーニング状態です (RECまたはTEC > 95)。\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_STF) != 0) {
        printf("  - Stuff error: スタッフビットのルール違反を検出しました。\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_FOR) != 0) {
        printf("  - Form error: フレームフォーマットの固定ビットが不正です。\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_ACK) != 0) {
        printf("  - Acknowledgment error: 送信メッセージに対しACKが返されませんでした。\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_BR) != 0) {
        printf("  - Bit recessive error: ドミナントビットを送信中にリセッシブを検出しました。\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_BD) != 0) {
        printf("  - Bit dominant error: リセッシブビットを送信中にドミナントを検出しました。\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_CRC) != 0) {
        printf("  - CRC error: 受信メッセージのCRCが一致しませんでした。\r\n");
    }

    // === 受信FIFOに関するエラー ===
    if ((can_error & HAL_CAN_ERROR_RX_FOV0) != 0) {
        printf("  - Rx FIFO0 overrun error: 受信FIFO0がオーバーランしました。\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_RX_FOV1) != 0) {
        printf("  - Rx FIFO1 overrun error: 受信FIFO1がオーバーランしました。\r\n");
    }

    // === 送信メールボックスに関するエラー ===
    if ((can_error & HAL_CAN_ERROR_TX_ALST0) != 0) {
        printf("  - TxMailbox 0 arbitration lost: 送信メールボックス0が調停に失敗しました。\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_TX_TERR0) != 0) {
        printf("  - TxMailbox 0 transmit error: 送信メールボックス0で送信エラーが発生しました。\r\n");
    }
    // メールボックス1と2も同様にチェック
    if ((can_error & HAL_CAN_ERROR_TX_ALST1) != 0) {
        printf("  - TxMailbox 1 arbitration lost\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_TX_TERR1) != 0) {
        printf("  - TxMailbox 1 transmit error\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_TX_ALST2) != 0) {
        printf("  - TxMailbox 2 arbitration lost\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_TX_TERR2) != 0) {
        printf("  - TxMailbox 2 transmit error\r\n");
    }

    // === HALドライバ内部のエラー ===
    if ((can_error & HAL_CAN_ERROR_TIMEOUT) != 0) {
        printf("  - Timeout error: 処理がタイムアウトしました。\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_NOT_INITIALIZED) != 0) {
        printf("  - Peripheral not initialized: ペリフェラルが初期化されていません。\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_NOT_READY) != 0) {
        printf("  - Peripheral not ready: ペリフェラルが準備完了状態ではありません。\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_NOT_STARTED) != 0) {
        printf("  - Peripheral not started: ペリフェラルが開始されていません。\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_PARAM) != 0) {
        printf("  - Parameter error: 関数の引数が不正です。\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_INTERNAL) != 0) {
        printf("  - Internal error: HALドライバの内部エラーです。\r\n");
    }

    printf("-----------------------------------------\r\n\r\n");
}

/**
 * @brief  現在動作中のCANペリフェラルのビットタイミング設定をレジスタから直接読み出し、
 * 計算されたボーレートなどと共にシリアルコンソールに出力します。
 * @param  hcan 確認対象のCANハンドル
 */
static void Print_CAN_BitTiming_Params(CAN_HandleTypeDef *const hcan) {
    uint32_t can_btr_reg;
    uint32_t pclk1_freq;

    // CAN_BTRレジスタの現在の値を読み出す
    can_btr_reg = hcan->Instance->BTR;

    // --- レジスタ値から各パラメータを抽出 ---
    // CAN_BTRレジスタのビット配置
    // | 31 | 30 | 29..26 | 25..24 | 23 | 22..20 | 19..16 | 15..10 | 9..0  |
    // | SILM | LBKM | -      | SJW    | -  | TS2    | TS1    | -      | BRP   |

    // ボーレートプリスケーラ (BRP)
    const uint32_t brp = (can_btr_reg & CAN_BTR_BRP);
    const uint32_t prescaler = brp + 1;

    // タイムセグメント1 (TS1)
    const uint32_t ts1 = (can_btr_reg & CAN_BTR_TS1) >> 16;
    const uint32_t time_seg1 = ts1 + 1;

    // タイムセグメント2 (TS2)
    const uint32_t ts2 = (can_btr_reg & CAN_BTR_TS2) >> 20;
    const uint32_t time_seg2 = ts2 + 1;

    // 再同期ジャンプ幅 (SJW)
    const uint32_t sjw = (can_btr_reg & CAN_BTR_SJW) >> 24;
    const uint32_t sync_jump_width = sjw + 1;

    // --- 計算 ---
    // APB1クロック周波数を取得 (CANペリフェラルのクロックソース)
    pclk1_freq = HAL_RCC_GetPCLK1Freq();

    // 1ビットあたりの合計タイムクアンタ数 (SYNC_SEGは常に1 Tq)
    const uint32_t total_tq = 1 + time_seg1 + time_seg2;

    // ボーレートの計算
    // BaudRate = PCLK1 / (Prescaler * (1 + TS1 + TS2))
    const uint32_t baud_rate = pclk1_freq / (prescaler * total_tq);

    // サンプルポイントの計算
    // SamplePoint = (1 + TS1) / (1 + TS1 + TS2) * 100
    const float sample_point = (float)(1 + time_seg1) * 100.0f / (float)total_tq;

    // --- 結果の表示 ---
    printf("--- Current CAN Bit Timing Parameters ---\r\n");
    printf("  [System]\r\n");
    printf("    APB1 Clock (PCLK1) : %lu Hz\r\n", pclk1_freq);
    printf("\r\n");
    printf("  [Register Raw Values (from CAN_BTR)]\r\n");
    printf("    BRP: %lu, TS1: %lu, TS2: %lu, SJW: %lu\r\n", brp, ts1, ts2, sjw);
    printf("\r\n");
    printf("  [Calculated Parameters]\r\n");
    printf("    Prescaler          : %lu\r\n", prescaler);
    printf("    Time Segment 1     : %lu Tq\r\n", time_seg1);
    printf("    Time Segment 2     : %lu Tq\r\n", time_seg2);
    printf("    Sync Jump Width    : %lu Tq\r\n", sync_jump_width);
    printf("    Total Time Quanta  : %lu Tq/bit\r\n", total_tq);
    printf("\r\n");
    printf("  [Result]\r\n");
    printf("    Calculated Baud Rate: %lu bps (%lu kbps)\r\n", baud_rate, baud_rate / 1000);
    printf("    Sample Point        : %.2f %%\r\n", sample_point);
    printf("-----------------------------------------\r\n\r\n");
}
#endif

/**
 * @brief リミットスイッチの状態を取得します。
 * @param limit_port リミットスイッチが接続されているGPIOポート
 * @param limit_pin リミットスイッチが接続されているGPIOピン
 * @param sw_type スイッチのタイプ (ノーマリオープンまたはノーマリクローズ)
 * @retval uint8_t スイッチの状態 (アクティブなら1, 非アクティブなら0)
 */
static uint8_t Robstride_get_switch_state(GPIO_TypeDef *const limit_port, const uint32_t limit_pin, const ROBSTRIDE_SWITCH_TYPE sw_type) {
    if (sw_type == ROBSTRIDE_SWITCH_NO) {                // ノーマリオープン (Normally Open) の場合
        return HAL_GPIO_ReadPin(limit_port, limit_pin);  // ピンがHighなら1 (アクティブ), Lowなら0 (非アクティブ)
    } else {                                             // ノーマリクローズ (Normally Close) の場合
        return !HAL_GPIO_ReadPin(limit_port, limit_pin); // ピンがLowなら1 (アクティブ), Highなら0 (非アクティブ)
    }
}

/**
 * @brief Robstrideの機械的位置をゼロに設定します。
 * @param dev_info Robstrideデバイス情報構造体へのポインタ
 * @retval なし
 */
static void Robstride_SetMechposToZero(Robstride_DeviceInfo *const dev_info, DelayFunction_t f_delay) {
    uint8_t data[8] = { 0x00 }; // 送信するデータ配列を初期化
    data[0] = 0x01;             // コマンドデータ (機械的位置をゼロにするための特定データ)
    // CANメッセージを送信 (CMD_SET_MECH_POSITION_TO_ZERO コマンド)
    while (1) {
        Robstride_SendBytes(dev_info->phcan, dev_info->device_id, CMD_SET_MECH_POSITION_TO_ZERO, dev_info->master_id, (uint8_t *)data, sizeof(data));
        f_delay(1); // 送信後に少し待機
        // フィードバックデータを読み取る (機械的位置がゼロに設定されたか確認)
        if (fabs(Read_Robstride_FeedbackData(dev_info).position - dev_info->ctrl_param.offset_pos) < 0.1f) {
            Robstride_ResetPositionTracking(dev_info);
            break; // 機械的位置がゼロに設定されたらループを抜ける
        }
    }
}

/**
 * @brief Robstrideのパラメータをプリセットします（PIDパラメータ設定、制御モード設定、フィードバック初期化）。Control_Disable状態でのみ設定可能です。
 * @param dev_info Robstrideデバイス情報構造体へのポインタ
 * @retval なし
 */
void Robstride_PresetParameters(Robstride_DeviceInfo *const dev_info, DelayFunction_t f_delay) {
    Robstride_SetPIDParams(dev_info, f_delay);                               // PIDパラメータを設定
    Robstride_SetControl(dev_info, dev_info->ctrl_param.ctrl_type, f_delay); // 制御モードを設定
    Robstride_fb_init(dev_info);                                             // フィードバックを初期化
}

/**
 * @brief RobstrideのPIDパラメータを設定します。Control_Disable状態でのみ設定可能です。
 * @param dev_info Robstrideデバイス情報構造体へのポインタ
 * @retval なし
 */
/* 起動時だけの有界読み書き確認。Type2は確認に使わず、対象アドレスの
 * 新着Type17と値の一致を要求する。失敗時はアームEnableを解禁しない。 */
static bool robstride_current_parameter(Robstride_DeviceInfo *dev, uint16_t address,
                                       float wanted, DelayFunction_t delay)
{
    bool ok=false;
    Robstride_BeginPriorityTransaction(dev->phcan);
    uint32_t seq=Robstride_GetParameterSequence(dev,address), start=HAL_GetTick();
    bool before_ok=Robstride_RequestReadParameterPriority(dev,address)==HAL_OK &&
        robstride_wait_for_parameter(dev,address,seq,start,delay);
    Robstride_FeedbackData fb=Read_Robstride_FeedbackData(dev);
    const float before=before_ok ? robstride_cached_parameter(&fb,address) : NAN;
    float after=NAN;
    for (unsigned attempt=0;attempt<3 && !ok;++attempt) {
        if (Robstride_WriteFloatDataPriority(dev,address,wanted)!=HAL_OK) continue;
        delay(2U);
        seq=Robstride_GetParameterSequence(dev,address);start=HAL_GetTick();
        if (Robstride_RequestReadParameterPriority(dev,address)==HAL_OK &&
            robstride_wait_for_parameter(dev,address,seq,start,delay)) {
            fb=Read_Robstride_FeedbackData(dev);after=robstride_cached_parameter(&fb,address);
            ok=isfinite(after) && fabsf(after-wanted)<0.00001f;
        }
    }
    Robstride_EndPriorityTransaction(dev->phcan);
    printf("[CurrentTune] ID=%u addr=0x%04x before=%.6f requested=%.6f readback=%.6f ok=%u\r\n",
           dev->device_id,address,(double)before,(double)wanted,(double)after,ok);
    return ok;
}

void Robstride_SetPIDParams(Robstride_DeviceInfo *const dev_info, DelayFunction_t f_delay) {
    Robstride_WriteFloatData(dev_info, ADDR_LOC_KP, dev_info->ctrl_param.pid.kp_pos);                  // 位置制御Pゲインを設定
    f_delay(10);                                                                                       // 書き込み後に少し待機
    Robstride_WriteFloatData(dev_info, ADDR_SPD_KP, dev_info->ctrl_param.pid.kp_vel);                  // 速度制御Pゲインを設定
    f_delay(10);                                                                                       // 書き込み後に少し待機
    Robstride_WriteFloatData(dev_info, ADDR_SPD_KI, dev_info->ctrl_param.pid.ki_vel);                  // 速度制御Iゲインを設定
    f_delay(10);                                                                                       // 書き込み後に少し待機
    Robstride_WriteFloatData(dev_info, ADDR_SPD_FILTER_GAIN, dev_info->ctrl_param.pid.filter_vel);     // 速度制御フィルタゲインを設定
    f_delay(10);                                                                                       // 書き込み後に少し待機
    bool ok=robstride_current_parameter(dev_info,ADDR_CURRENT_KP,dev_info->ctrl_param.pid.kp_cur,f_delay);
    ok=robstride_current_parameter(dev_info,ADDR_CURRENT_KI,dev_info->ctrl_param.pid.ki_cur,f_delay) && ok;
    ok=robstride_current_parameter(dev_info,ADDR_CURRENT_FILTER_GAIN,dev_info->ctrl_param.pid.filter_cur,f_delay) && ok;
    if (dev_info->device_id==1U || dev_info->device_id==2U) arm_current_config_ok[dev_info->device_id]=ok;
}

/**
 * @brief Robstrideの速度制限を設定します。Control_Disable時にのみ有効です。
 * @param dev_info Robstrideデバイス情報構造体へのポインタ
 * @retval なし
 */
void Robstride_SetVelocityLimit(Robstride_DeviceInfo *const dev_info, DelayFunction_t f_delay) {
    switch (dev_info->device) {                                                            // デバイスの種類によって制限値の上限が異なる
        case Robstride_02:                                                                 // Robstride_02 の場合
            if (dev_info->ctrl_param.velocity_limit == ROBSTRIDE_VELOCITY_LIMIT_DISABLE) { // 速度制限が無効の場合
                dev_info->ctrl_param.velocity_limit_size = 44.0f;                          // デフォルトの最大制限値を設定
            } else {                                                                       // 速度制限が有効の場合
                // 設定値を 0.0f と 44.0f の間にクリッピング
                dev_info->ctrl_param.velocity_limit_size = fmaxf(fminf(dev_info->ctrl_param.velocity_limit_size, 44.0f), 0.0f);
            }
            break;
        case Robstride_04:                                                                 // Robstride_04 の場合
            if (dev_info->ctrl_param.velocity_limit == ROBSTRIDE_VELOCITY_LIMIT_DISABLE) { // 速度制限が無効の場合
                dev_info->ctrl_param.velocity_limit_size = 15.0f;                          // デフォルトの最大制限値を設定
            } else {                                                                       // 速度制限が有効の場合
                // 設定値を 0.0f と 15.0f の間にクリッピング
                dev_info->ctrl_param.velocity_limit_size = fmaxf(fminf(dev_info->ctrl_param.velocity_limit_size, 15.0f), 0.0f);
            }
            break;
        case Robstride_05_Edu:
            if (dev_info->ctrl_param.velocity_limit == ROBSTRIDE_VELOCITY_LIMIT_DISABLE) {
                dev_info->ctrl_param.velocity_limit_size = 50.0f; // Max Speed: 50 rad/s
            } else {
                // 設定値を 0.0f と 50.0f の間にクリッピング
                dev_info->ctrl_param.velocity_limit_size = fmaxf(fminf(dev_info->ctrl_param.velocity_limit_size, 50.0f), 0.0f);
            }
            break;
            // defaultケースが抜けているため、Robstride_04以外のデバイスタイプの場合、処理がスキップされる。
        default:
            break;
    }
    while (1) {
        Robstride_WriteFloatData(dev_info, ADDR_LIMIT_SPEED, dev_info->ctrl_param.velocity_limit_size);                  // 速度制限値を書き込み
        Robstride_RequestReadParameter(dev_info, ADDR_LIMIT_SPEED);                                                      // 書き込み後に読み出し要求を送信して、設定が反映されたか確認
        f_delay(1);                                                                                                      // 書き込み後に少し待機
        if (fabsf(Read_Robstride_FeedbackData(dev_info).limit_spd - dev_info->ctrl_param.velocity_limit_size) < 0.01f) { // 設定が反映されたか確認
            break;                                                                                                       // 反映されたらループを抜ける
        }
    }
}

/**
 * @brief Robstrideの電流制限を設定します。Control_Disable時にのみ有効です。
 * @param dev_info Robstrideデバイス情報構造体へのポインタ
 * @retval なし
 */
void Robstride_SetCurrentLimit(Robstride_DeviceInfo *const dev_info, DelayFunction_t f_delay) {
    switch (dev_info->device) {                                                          // デバイスの種類によって制限値の上限が異なる
        case Robstride_02:                                                               // Robstride_02 の場合
            if (dev_info->ctrl_param.current_limit == ROBSTRIDE_CURRENT_LIMIT_DISABLE) { // 電流制限が無効の場合
                dev_info->ctrl_param.current_limit_size = 23.0f;                         // デフォルトの最大制限値を設定
            } else {                                                                     // 電流制限が有効の場合
                // 設定値を 0.0f と 23.0f の間にクリッピング
                dev_info->ctrl_param.current_limit_size = fmaxf(fminf(dev_info->ctrl_param.current_limit_size, 23.0f), 0.0f);
            }
            break;
        case Robstride_04: // Robstride_04 の場合
            if (dev_info->ctrl_param.current_limit == ROBSTRIDE_CURRENT_LIMIT_DISABLE) {
                // 電流制限が無効の場合
                // TODO: 注意: ここでは velocity_limit_size が使われているが、current_limit_size の誤りと思われる。
                dev_info->ctrl_param.velocity_limit_size = 90.0f; // デフォルトの最大制限値を設定
            } else {
                // 電流制限が有効の場合
                // TODO: 注意: ここでは velocity_limit_size が使われているが、current_limit_size の誤りと思われる。
                // 設定値を 0.0f と 90.0f の間にクリッピング
                dev_info->ctrl_param.velocity_limit_size = fmaxf(fminf(dev_info->ctrl_param.current_limit_size, 90.0f), 0.0f);
            }
            break;
        case Robstride_05_Edu:
            if (dev_info->ctrl_param.current_limit == ROBSTRIDE_CURRENT_LIMIT_DISABLE) {
                dev_info->ctrl_param.current_limit_size = 11.0f; // Max Current: 11 A
            } else {
                dev_info->ctrl_param.current_limit_size = fmaxf(fminf(dev_info->ctrl_param.current_limit_size, 11.0f), 0.0f);
            }
            break;
            // defaultケースが抜けているため、Robstride_04以外のデバイスタイプの場合、処理がスキップされる。
        default:
            break;
    }
    while (1) {
        Robstride_WriteFloatData(dev_info, ADDR_LIMIT_CURRENT, dev_info->ctrl_param.current_limit_size); // 電流制限値を書き込み
        Robstride_RequestReadParameter(dev_info, ADDR_LIMIT_CURRENT);                                    // 書き込み後に読み出し要求を送信して、設定が反映されたか確認
        f_delay(1);                                                                                      // 書き込み後に少し待機
        if (fabsf(Read_Robstride_FeedbackData(dev_info).limit_cur - dev_info->ctrl_param.current_limit_size) < 0.01f) {
            // 設定が反映されたか確認
            break; // 反映されたらループを抜ける
        }
    }
}

/**
 * @brief Robstrideのトルク制限を設定します。Control_Disable時にのみ有効です。
 * @param dev_info Robstrideデバイス情報構造体へのポインタ
 * @retval なし
 */
void Robstride_SetTorqueLimit(Robstride_DeviceInfo *const dev_info) {
    switch (dev_info->device) {                                                        // デバイスの種類によって制限値の上限が異なる
        case Robstride_02:                                                             // Robstride_02 の場合
            if (dev_info->ctrl_param.torque_limit == ROBSTRIDE_TORQUE_LIMIT_DISABLE) { // トルク制限が無効の場合
                dev_info->ctrl_param.torque_limit_size = 17.0f;                        // デフォルトの最大制限値を設定
            } else {                                                                   // トルク制限が有効の場合
                // 設定値を 0.0f と 17.0f の間にクリッピング
                dev_info->ctrl_param.torque_limit_size = fmaxf(fminf(dev_info->ctrl_param.torque_limit_size, 17.0f), 0.0f);
            }
            break;
        case Robstride_04:                                                             // Robstride_04 の場合
            if (dev_info->ctrl_param.torque_limit == ROBSTRIDE_TORQUE_LIMIT_DISABLE) { // トルク制限が無効の場合
                dev_info->ctrl_param.torque_limit_size = 120.0f;                       // デフォルトの最大制限値を設定
            } else {                                                                   // トルク制限が有効の場合
                // 設定値を 0.0f と 120.0f の間にクリッピング
                dev_info->ctrl_param.torque_limit_size = fmaxf(fminf(dev_info->ctrl_param.torque_limit_size, 120.0f), 0.0f);
            }
            break;
        case Robstride_05_Edu:
            if (dev_info->ctrl_param.torque_limit == ROBSTRIDE_TORQUE_LIMIT_DISABLE) {
                dev_info->ctrl_param.torque_limit_size = 6.0f; // Max Torque: 6 Nm
            } else {
                dev_info->ctrl_param.torque_limit_size = fmaxf(fminf(dev_info->ctrl_param.torque_limit_size, 6.0f), 0.0f);
            }
            break;
            // defaultケースが抜けているため、Robstride_04以外のデバイスタイプの場合、処理がスキップされる。
        default:
            break;
    }
    Robstride_WriteFloatData(dev_info, ADDR_LIMIT_TORQUE, dev_info->ctrl_param.torque_limit_size); // トルク制限値を書き込み
    // TODO: Robstride_SetVelocityLimit(), SetCurrentLimit() のように書き込みを確認しなくてもよいのか？
}

/**
 * @brief Robstrideのキャリブレーション（原点出し）を実行します。
 * @param device_info Robstrideデバイス情報構造体へのポインタ
 * @param calib_velocity キャリブレーション時の速度
 * @param sw_type リミットスイッチのタイプ
 * @param limit_port リミットスイッチのGPIOポート
 * @param limit_pin リミットスイッチのGPIOピン
 * @retval なし
 */
void Robstride_Calibration(Robstride_DeviceInfo *const device_info, float calib_velocity, const ROBSTRIDE_SWITCH_TYPE sw_type, GPIO_TypeDef *const limit_port, const uint32_t limit_pin, DelayFunction_t f_delay) {
    switch (device_info->ctrl_param.use_internal_offset) {
        case ROBSTRIDE_USE_OFFSET_POS_INTERNAL:               // 内部オフセットを使用する場合
            return;                                           // キャリブレーション不要
        case ROBSTRIDE_USE_OFFSET_POS_INITIAL:                // 初期位置をオフセットとして使用する場合
            Robstride_SetMechposToZero(device_info, f_delay); // 現在の機械的位置をゼロに設定
            return;                                           // キャリブレーション完了
        case ROBSTRIDE_USE_OFFSET_POS_CALIB:                  // キャリブレーションによりオフセットを設定する場合
            break;                                            // キャリブレーション処理へ進む
        default:                                              // その他の場合
            return;                                           // 何もしない
    }
    Robstride_SetControl(device_info, ROBSTRIDE_CTRL_VEL, f_delay); // 制御モードを速度制御に設定
    Robstride_ControlEnable(device_info, f_delay);                  // モータ制御を有効化
    if (device_info->ctrl_param.rotation == ROBSTRIDE_ROT_CW) {     // 回転方向が時計回り(CW)の場合
        calib_velocity *= -1;                                       // キャリブレーション速度を反転
    }
    Robstride_SetTarget(device_info, calib_velocity); // 目標速度を設定
    while (!Robstride_get_switch_state(limit_port, limit_pin, sw_type)) {
        f_delay(1);
    } // リミットスイッチが押されるまで待機
    Robstride_ControlDisable(device_info, f_delay);   // モータ制御を無効化
    Robstride_SetMechposToZero(device_info, f_delay); // 現在の機械的位置をゼロに設定 (原点確定)
    return;
}

/**
 * @brief Robstrideの制御モードを設定します。
 * この関数はPIDパラメータなどの再初期化は行いません。
 * 制御モード変更時にPIDパラメータをリセットしたい場合は Robstride_ChangeControl を使用してください。
 * @param dev_info Robstrideデバイス情報構造体へのポインタ
 * @param new_ctrl_type 新しい制御モード
 * @retval なし
 */
static void robstride_set_control_internal(
    Robstride_DeviceInfo *const dev_info,
    const ROBSTRIDE_CTRL_TYPE new_ctrl_type,
    DelayFunction_t f_delay,
    const bool enable_after_write)
{
    if (dev_info == NULL || f_delay == NULL) {
        return;
    }

    const uint8_t wire_ctrl_type = robstride_wire_control_type(new_ctrl_type);
    dev_info->ctrl_param._mode_configured = 0U;
    /*
     * ctrl_type は設定値であり、モーター側の現在値ではない。起動時には
     * 同じ値でも必ず CAN 経由で書き込んで、電源投入直後のモーターへ反映する。
     */
    if (!Robstride_ControlDisable(dev_info, f_delay)) {                       // control_disable時のみ制御モードが変更可能
        return;
    }
    dev_info->ctrl_param.ctrl_type = new_ctrl_type;                           // 新しい制御モードを設定
    if (!robstride_write_int_verified(dev_info,
                                      ADDR_RUN_MODE,
                                      (uint16_t)wire_ctrl_type,
                                      wire_ctrl_type,
                                      f_delay)) {
        dev_info->ctrl_param._enable_flag = 0U;
        return;
    }
    dev_info->ctrl_param._mode_configured = 1U;
    if (enable_after_write) {
        (void)Robstride_ControlEnable(dev_info, f_delay);                     // モータ制御を有効化
    }
}

void Robstride_SetControl(Robstride_DeviceInfo *const dev_info,
                          const ROBSTRIDE_CTRL_TYPE new_ctrl_type,
                          DelayFunction_t f_delay)
{
    robstride_set_control_internal(dev_info, new_ctrl_type, f_delay, true);
}

uint8_t Robstride_SetControlDisabled(Robstride_DeviceInfo *const dev_info,
                                     const ROBSTRIDE_CTRL_TYPE new_ctrl_type,
                                     DelayFunction_t f_delay)
{
    robstride_set_control_internal(dev_info, new_ctrl_type, f_delay, false);
    return (dev_info != NULL &&
            dev_info->ctrl_param._mode_configured != 0U) ? 1U : 0U;
}

/**
 * @brief Robstrideの制御モードを変更し、関連する制御パラメータを初期化します。
 * @param dev_info Robstrideデバイス情報構造体へのポインタ
 * @param new_ctrl_type 新しい制御モード
 * @retval なし
 */
void Robstride_ChangeControl(Robstride_DeviceInfo *const dev_info, const ROBSTRIDE_CTRL_TYPE new_ctrl_type, DelayFunction_t f_delay) {
    const uint8_t wire_ctrl_type = robstride_wire_control_type(new_ctrl_type);
    Robstride_Ctrl_Struct_init(&(dev_info->ctrl_param)); // 制御パラメータ構造体を初期化 (PIDパラメータなども初期値に戻る)
    if (!Robstride_ControlDisable(dev_info, f_delay)) {  // control_disable時のみ制御モードが変更可能
        return;
    }
    dev_info->ctrl_param.ctrl_type = new_ctrl_type;      // 新しい制御モードを設定
    if (!robstride_write_int_verified(dev_info,
                                      ADDR_RUN_MODE,
                                      (uint16_t)wire_ctrl_type,
                                      wire_ctrl_type,
                                      f_delay)) {
        dev_info->ctrl_param._enable_flag = 0U;
    } else {
        dev_info->ctrl_param._mode_configured = 1U;
    }
}

/**
 * @brief Robstrideの目標値を設定します。
 * @param device_info Robstrideデバイス情報構造体へのポインタ
 * @param target_value 設定する目標値
 * @retval なし
 */
static HAL_StatusTypeDef robstride_set_target_internal(
    Robstride_DeviceInfo *const device_info,
    const float target_value,
    const bool priority,
    const bool generation_guarded,
    const uint32_t expected_generation)
{
    uint16_t address;
    float wire_value;

    if (device_info == NULL || !isfinite(target_value)) return HAL_ERROR;
    if (generation_guarded &&
        (!device_info->ctrl_param._enable_flag ||
         Read_Robstride_FeedbackData(device_info).mode_status != ROBSTRIDE_STATE_ENABLE))
        return HAL_BUSY;
    /* MPC/DOBにもサービス世代・優先トランザクションのゲートを適用する。
     * 最適化計算中は割り込みを止めず、送信直前に再照合する。 */
    if ((generation_guarded &&
         device_info->ctrl_param._target_generation != expected_generation) ||
        (!priority && Robstride_IsPriorityTransactionActive(device_info->phcan)))
        return HAL_BUSY;

    if (device_info && device_info->ctrl_param.ctrl_type == ROBSTRIDE_CTRL_POS_MPC &&
        (!ArmPositionMpc_TargetAllowedForDevice(device_info->device_id, target_value) || device_info->device_id<1U || device_info->device_id>2U)) {
        if(device_info->device_id>=1U && device_info->device_id<=2U) {
            arm_guard_tripped[device_info->device_id]=true;
            ArmPositionMpc_Reset(&arm_position_mpc[device_info->device_id]);
        }
        return robstride_send_current(device_info,0.0f);
    }
    if (device_info == NULL || !isfinite(target_value)) {
        return HAL_ERROR;
    }
    if (device_info->ctrl_param._enable_flag != 0U &&
        !Robstride_ArmGuardCheck(device_info)) return HAL_ERROR;
    device_info->ctrl_param._target_value = target_value;

    /* VEL_DOBの速度目標はF7に保持し、速度レジスタへは送らない。 */
    if (device_info->ctrl_param.ctrl_type == ROBSTRIDE_CTRL_VEL_DOB ||
        device_info->ctrl_param.ctrl_type == ROBSTRIDE_CTRL_POS_MPC) {
        if (device_info->ctrl_param._enable_flag == 0U) {
            if(device_info->device_id>=1U && device_info->device_id<=2U)
                ArmPositionMpc_Reset(&arm_position_mpc[device_info->device_id]);
            device_info->ctrl_param._req_value = 0.0f;
            Robstride_Actuator_VelocityDob_Reset(
                &(device_info->ctrl_param.velocity_dob_state));
            return HAL_OK;
        }

        const Robstride_FeedbackData feedback =
            Read_Robstride_FeedbackData(device_info);
        if (feedback.get_flag == 0U || !isfinite(feedback.velocity)) {
            device_info->ctrl_param._req_value = 0.0f;
            Robstride_Actuator_VelocityDob_Reset(
                &(device_info->ctrl_param.velocity_dob_state));
            return robstride_send_current(device_info, 0.0f);
        }

        float velocity_for_dob = feedback.velocity;
        if (device_info->device_id == 1U || device_info->device_id == 2U) {
            Robstride_StandardFeedback standard;
            if (!Robstride_ReadStandardFeedback(device_info,&standard) ||
                (uint32_t)(HAL_GetTick()-standard.tick)>50U || !isfinite(standard.velocity))
                return robstride_send_current(device_info,0.0f);
            /* 指定により両軸40ms固定。根本はこのフィルタで調整する。 */
            velocity_for_dob=ArmVelocityEstimate_UpdateVelocityWithTau(
                &arm_velocity_estimate[device_info->device_id],standard.velocity,standard.tick,
                .04f);
        }
        float velocity_reference=target_value;
        if(device_info->ctrl_param.ctrl_type == ROBSTRIDE_CTRL_POS_MPC) {
            Robstride_StandardFeedback standard;
            if(!Robstride_ReadStandardFeedback(device_info,&standard) ||
               HAL_GetTick()-standard.tick>50U || !ArmPositionMpc_TargetAllowedForDevice(device_info->device_id, standard.position))
                return robstride_send_current(device_info,0.0f);
            velocity_reference=ArmPositionMpc_UpdateForDevice(device_info->device_id, &arm_position_mpc[device_info->device_id],
                standard.position,velocity_for_dob,target_value,
                device_info->ctrl_param.velocity_dob.reference_alpha,
                device_info->ctrl_param.velocity_dob.control_period);
        }
        const float current = Robstride_Actuator_VelocityDob_Update(
            &(device_info->ctrl_param.velocity_dob),
            &(device_info->ctrl_param.velocity_dob_state),
            velocity_reference,
            velocity_for_dob,
            device_info->ctrl_param.velocity_dob.control_period);
        const uint32_t send_mask = __get_PRIMASK();
        __disable_irq();
        if ((generation_guarded && device_info->ctrl_param._target_generation != expected_generation) ||
            !device_info->ctrl_param._enable_flag ||
            Robstride_IsPriorityTransactionActive(device_info->phcan)) {
            __set_PRIMASK(send_mask);
            return HAL_BUSY;
        }
        const HAL_StatusTypeDef sent = robstride_send_current(device_info, current);
        __set_PRIMASK(send_mask);
        const uint8_t id = device_info->device_id;
        const uint32_t now = HAL_GetTick();
        if (id==1U) {
            arm_fast_capture_started=true;
            Robstride_StandardFeedback standard;
            if (Robstride_ReadStandardFeedback(device_info,&standard)) {
                const ArmFastSample sample={now,standard.tick,standard.position,
                    standard.velocity,velocity_for_dob,device_info->ctrl_param._req_value,
                    Robstride_StandardReferenceCurrent(device_info,&standard),target_value};
                const uint32_t count=arm_fast_log.count;
                arm_fast_log.samples[count%2048U]=sample;
                __DMB();
                arm_fast_log.count=count+1U;
            }
        }
        if ((id==1U || id==2U) && (uint32_t)(now-arm_dob_log_tick[id])>=20U) {
            Robstride_StandardFeedback standard;
            if (Robstride_ReadStandardFeedback(device_info,&standard)) {
                const ArmDobSample sample = {now,id,velocity_reference,
                    device_info->ctrl_param.velocity_dob_state.omega_model /
                        device_info->ctrl_param.velocity_dob.velocity_unit_to_rad_s,
                    velocity_for_dob,device_info->ctrl_param._req_value,standard.position,
                    Robstride_StandardReferenceCurrent(device_info,&standard)};
                arm_dob_log.samples[arm_dob_log.count%4096U] = sample;
                ++arm_dob_log.count;
                arm_dob_log_tick[id]=now;
            }
        }
        return sent;
    }

    if (!robstride_target_parameter(device_info,
                                    target_value,
                                    &address,
                                    &wire_value)) {
        robstride_trip_position_guard(device_info);
        return HAL_ERROR;
    }

    /*
     * Service/control traffic owns the CAN path.  A normal target that races
     * with a service must never be released after the service has completed.
     * The guarded path also checks the command generation immediately before
     * queueing, so invalidation and queue insertion cannot be reordered.
     */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool generation_matches =
        !generation_guarded ||
        (device_info->ctrl_param._target_generation == expected_generation);
    const bool transaction_active =
        (!priority && Robstride_IsPriorityTransactionActive(device_info->phcan));
    if (!generation_matches || transaction_active) {
        __set_PRIMASK(primask);
        return HAL_BUSY;
    }

    device_info->ctrl_param._target_value = target_value;

    if (priority) {
        const HAL_StatusTypeDef result = Robstride_WriteFloatDataPriority(
            device_info, address, wire_value);
        if ((result == HAL_OK) &&
            (device_info->ctrl_param.ctrl_type == ROBSTRIDE_CTRL_POS)) {
            device_info->ctrl_param._position_target_wire = wire_value;
            device_info->ctrl_param._position_target_valid = 1U;
        }
        __set_PRIMASK(primask);
        return result;
    }

    const HAL_StatusTypeDef sent = Robstride_WriteFloatData(device_info, address, wire_value);
    if (sent == HAL_OK && device_info->ctrl_param.ctrl_type == ROBSTRIDE_CTRL_POS) {
        device_info->ctrl_param._position_target_wire = wire_value;
        device_info->ctrl_param._position_target_valid = 1U;
    }
    __set_PRIMASK(primask);
    return sent;
}

HAL_StatusTypeDef Robstride_SetTarget(Robstride_DeviceInfo *const device_info,
                                      const float target_value) {
    return robstride_set_target_internal(device_info,
                                         target_value,
                                         false,
                                         false,
                                         0U);
}

HAL_StatusTypeDef Robstride_SetTargetIfGeneration(
    Robstride_DeviceInfo *const device_info,
    const float target_value,
    const uint32_t expected_generation)
{
    return robstride_set_target_internal(device_info,
                                         target_value,
                                         false,
                                         true,
                                         expected_generation);
}

uint32_t Robstride_GetTargetGeneration(
    const Robstride_DeviceInfo *const device_info)
{
    if (device_info == NULL) {
        return 0U;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint32_t generation = device_info->ctrl_param._target_generation;
    __set_PRIMASK(primask);
    return generation;
}

void Robstride_InvalidateTargetGeneration(
    Robstride_DeviceInfo *const device_info)
{
    if (device_info == NULL) {
        return;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    ++device_info->ctrl_param._target_generation;
    if (device_info->ctrl_param._target_generation == 0U) {
        /* 0は初期値として予約し、世代の一致判定を壊さない。 */
        device_info->ctrl_param._target_generation = 1U;
    }
    __set_PRIMASK(primask);
}

static bool robstride_set_target_verified(Robstride_DeviceInfo *const device_info,
                                          const float target_value,
                                          DelayFunction_t f_delay)
{
    uint16_t address;
    float wire_value;
    bool success = false;
    const uint32_t transaction_start = HAL_GetTick();

    if (device_info == NULL || !isfinite(target_value)) {
        return false;
    }

    /* VEL_DOBは速度目標をF7側で処理するため、切替時は電流指令を0にする。 */
    if (device_info->ctrl_param.ctrl_type == ROBSTRIDE_CTRL_VEL_DOB ||
        device_info->ctrl_param.ctrl_type == ROBSTRIDE_CTRL_POS_MPC) {
        device_info->ctrl_param._target_value = target_value;
        device_info->ctrl_param._req_value = 0.0f;
        Robstride_Actuator_VelocityDob_Reset(
            &(device_info->ctrl_param.velocity_dob_state));
        return Robstride_WriteFloatDataPriority(device_info,
                                                ADDR_IQ_REF,
                                                0.0f) == HAL_OK;
    }

    if (!robstride_target_parameter(device_info,
                                    target_value,
                                    &address,
                                    &wire_value)) {
        robstride_trip_position_guard(device_info);
        return false;
    }
    Robstride_BeginPriorityTransaction(device_info->phcan);
    while (!robstride_deadline_reached(transaction_start,
                                       ROBSTRIDE_SERVICE_TIMEOUT_MS)) {
        Robstride_ClearPriorityTxQueue(device_info->phcan);

        const uint32_t feedback_sequence =
            Robstride_GetFeedbackSequence(device_info);
        if (Robstride_WriteFloatDataPriority(device_info,
                                             address,
                                             wire_value) == HAL_OK &&
            robstride_wait_for_feedback(device_info,
                                        feedback_sequence,
                                        HAL_GetTick(),
                                        f_delay)) {
            const uint32_t parameter_sequence =
                Robstride_GetParameterSequence(device_info, address);
            if (Robstride_RequestReadParameterPriority(device_info, address) == HAL_OK &&
                robstride_wait_for_parameter(device_info,
                                             address,
                                             parameter_sequence,
                                             HAL_GetTick(),
                                             f_delay)) {
                const Robstride_FeedbackData feedback =
                    Read_Robstride_FeedbackData(device_info);
                const float applied = robstride_cached_parameter(&feedback,
                                                                 address);
                if (isfinite(applied) &&
                    fabsf(applied - wire_value) <= ROBSTRIDE_PARAMETER_TOLERANCE) {
                    success = true;
                    break;
                }
            }
        }
        f_delay(1U);
    }

    Robstride_ClearPriorityTxQueue(device_info->phcan);
    Robstride_EndPriorityTransaction(device_info->phcan);
    if (success) {
        device_info->ctrl_param._target_value = target_value;
        if (device_info->ctrl_param.ctrl_type == ROBSTRIDE_CTRL_POS) {
            device_info->ctrl_param._position_target_wire = wire_value;
            device_info->ctrl_param._position_target_valid = 1U;
        }
    }
    return success;
}

/**
 * @brief Robstrideのモータ制御を有効にします。
 * @param dev_info Robstrideデバイス情報構造体へのポインタ
 * @retval なし
 */
uint8_t Robstride_ControlEnable(Robstride_DeviceInfo *const dev_info, DelayFunction_t f_delay) {
    if (dev_info == NULL || f_delay == NULL) {
        return 0U;
    }
    if (!Robstride_ArmGuardCheck(dev_info)) {
        printf("[ArmGuard] ID %u enable blocked: range/stale/latch\r\n", dev_info->device_id);
        return 0U;
    }
    if ((dev_info->device_id==1U || dev_info->device_id==2U) && !arm_current_config_ok[dev_info->device_id]) {
        printf("[CurrentTune] ID %u enable blocked: current parameter verify failed\r\n",dev_info->device_id);
        return 0U;
    }
    if (dev_info->ctrl_param._mode_configured == 0U) {
        /* 一部機種ではrun_modeの読み出し応答が返らない。Type 2の
         * mode_statusをEnableの成否として使い、読み出し失敗だけで
         * Enableを遮断しない。 */
        printf("[Robstride] ID %u Enable warning: run_mode not verified; proceeding\r\n",
               (unsigned int)dev_info->device_id);
    }
    printf("[Robstride] ID %u Enable request\r\n",
           (unsigned int)dev_info->device_id);
    /* 境界超過後は、古いloc_refを残したまま再Enableしない。 */
    if ((dev_info->ctrl_param.ctrl_type == ROBSTRIDE_CTRL_POS) &&
        robstride_feedback_position_out_of_range(dev_info)) {
        robstride_trip_position_guard(dev_info);
        dev_info->ctrl_param._enable_flag = 0U;
        return 0U;
    }

    /*
     * Disable中もモータ側のloc_refは保持されるため、Enableを先に送ると
     * 前回の目標へ一気に追従する。位置制御では現在位置を保持目標として
     * 優先キューへ検証書込みし、成功してからEnableする。
    */
    if (dev_info->ctrl_param.ctrl_type == ROBSTRIDE_CTRL_POS) {
        /*
         * Enable直前はモータ軸が手で動かされた可能性もあるため、
         * 前回のloc_refを周回選択の基準にしない。最新フィードバックを
         * 基準にして、現在位置をそのまま保持目標へ書き込む。
         */
        dev_info->ctrl_param._position_target_valid = 0U;
        const Robstride_FeedbackData feedback =
            Read_Robstride_FeedbackData(dev_info);
        if ((feedback.get_flag == 0U) || !isfinite(feedback.position) ||
            !robstride_set_target_verified(dev_info,
                                           feedback.position,
                                           f_delay)) {
            dev_info->ctrl_param._enable_flag = 0U;
            return 0U;
        }
    }

    const uint8_t success = robstride_control_command_verified(
        dev_info,
        CMD_ENABLE,
        ROBSTRIDE_STATE_ENABLE,
        f_delay);
    dev_info->ctrl_param._enable_flag = (success != 0U) ? 1U : 0U;
    if (success != 0U) dev_info->ctrl_param._position_guard_latched = 0U;
    {
        const Robstride_FeedbackData feedback = Read_Robstride_FeedbackData(dev_info);
        printf("[Robstride] ID %u Enable result=%u sw=%u feedback=%u mode=%u run_mode=%u\r\n",
               (unsigned int)dev_info->device_id,
               (unsigned int)success,
               (unsigned int)dev_info->ctrl_param._enable_flag,
               (unsigned int)feedback.get_flag,
               (unsigned int)feedback.mode_status,
               (unsigned int)feedback.run_mode);
    }
    return success;
}

/**
 * @brief Robstrideのモータ制御を無効（リセット）します。
 * @param dev_info Robstrideデバイス情報構造体へのポインタ
 * @retval なし
 */
uint8_t Robstride_ControlDisable(Robstride_DeviceInfo *const dev_info, DelayFunction_t f_delay) {
    if(!dev_info || !f_delay) return 0U;
    if (dev_info->device_id>=1U && dev_info->device_id<=2U)
        ArmPositionMpc_Reset(&arm_position_mpc[dev_info->device_id]);
    if (dev_info->device_id == 1U || dev_info->device_id == 2U)
        arm_velocity_estimate[dev_info->device_id]=(ArmVelocityEstimate){0};
    dev_info->ctrl_param._req_value = 0.0f;
    Robstride_Actuator_VelocityDob_Reset(&(dev_info->ctrl_param.velocity_dob_state));
    /* Disable時点より前に取得したROS指令を再Enable後へ持ち越さない。 */
    Robstride_InvalidateTargetGeneration(dev_info);
    const uint8_t success = robstride_control_command_verified(
        dev_info,
        CMD_RESET,
        ROBSTRIDE_STATE_DISABLE,
        f_delay);
    /* タイムアウト時も以後の目標値送信を止める。 */
    dev_info->ctrl_param._enable_flag = 0U;
    dev_info->ctrl_param._position_target_valid = 0U;
    return success;
}

uint8_t Robstride_ServiceChangeControl(Robstride_DeviceInfo *const dev_info,
                                       const ROBSTRIDE_CTRL_TYPE new_ctrl_type,
                                       DelayFunction_t f_delay)
{
    if (dev_info == NULL || f_delay == NULL) {
        return 0U;
    }

    const uint8_t wire_ctrl_type = robstride_wire_control_type(new_ctrl_type);

    if (new_ctrl_type < ROBSTRIDE_CTRL_POS ||
        (new_ctrl_type > ROBSTRIDE_CTRL_CURRENT &&
         new_ctrl_type != ROBSTRIDE_CTRL_VEL_DOB &&
         new_ctrl_type != ROBSTRIDE_CTRL_POS_MPC)) {
        return 0U;
    }

    /* Preserve the state before ControlDisable clears the software flag. */
    const bool was_enabled =
        (dev_info->ctrl_param._enable_flag != 0U) &&
        (Read_Robstride_FeedbackData(dev_info).mode_status == ROBSTRIDE_STATE_ENABLE);
    const float hold_position = Read_Robstride_FeedbackData(dev_info).position;
    uint8_t success = 0U;

    /* Keep latest-value Type 1 and periodic Get traffic fenced out for the
     * complete multi-step mode transaction, not only for each individual
     * CAN request.  The nested service helpers share this transaction. */
    dev_info->ctrl_param._mode_configured = 0U;
    Robstride_BeginPriorityTransaction(dev_info->phcan);
    if (!Robstride_ControlDisable(dev_info, f_delay)) {
        const Robstride_FeedbackData feedback = Read_Robstride_FeedbackData(dev_info);
        printf("[Robstride] ID %u mode change disable failed: mode=%u run_mode=%u\r\n",
               (unsigned int)dev_info->device_id,
               (unsigned int)feedback.mode_status,
               (unsigned int)feedback.run_mode);
        goto service_complete;
    }

    dev_info->ctrl_param.ctrl_type = new_ctrl_type;
    const float safe_target = (new_ctrl_type == ROBSTRIDE_CTRL_POS || new_ctrl_type == ROBSTRIDE_CTRL_POS_MPC)
                                  ? hold_position
                                  : 0.0f;

    /* Type 18 has only a Type 2 response, so the write is followed by a
     * Type 17 read of 0x7005 and an exact value check. */
    if (!robstride_write_int_verified(dev_info,
                                      ADDR_RUN_MODE,
                                      (uint16_t)wire_ctrl_type,
                                      wire_ctrl_type,
                                      f_delay)) {
        const Robstride_FeedbackData feedback = Read_Robstride_FeedbackData(dev_info);
        printf("[Robstride] ID %u mode change run_mode failed: requested=%u actual=%u\r\n",
               (unsigned int)dev_info->device_id,
               (unsigned int)wire_ctrl_type,
               (unsigned int)feedback.run_mode);
        dev_info->ctrl_param._enable_flag = 0U;
        goto service_complete;
    }
    dev_info->ctrl_param._mode_configured = 1U;

    /* Enable中の切替だけ、現在位置を保持目標として設定する。Disable中は
     * 次のROS指令を待つため、古い目標値の書き込み自体を行わない。 */
    if (was_enabled &&
        !robstride_set_target_verified(dev_info, safe_target, f_delay)) {
        printf("[Robstride] ID %u mode change target failed: target=%.6f\r\n",
               (unsigned int)dev_info->device_id,
               (double)safe_target);
        dev_info->ctrl_param._enable_flag = 0U;
        goto service_complete;
    }

    if (was_enabled && !Robstride_ControlEnable(dev_info, f_delay)) {
        printf("[Robstride] ID %u mode change re-enable failed\r\n",
               (unsigned int)dev_info->device_id);
        dev_info->ctrl_param._enable_flag = 0U;
        goto service_complete;
    }

    success = 1U;

service_complete:
    Robstride_EndPriorityTransaction(dev_info->phcan);
    return success;
}

void Robstride_CheckActiveReportStatus(Robstride_DeviceInfo *const device_info) {
    Robstride_RequestReadParameter(device_info, ADDR_EPSCAN_TIME);
}

/**
 * @brief モーターに保存されている全ての読み取り可能なパラメータを要求します。
 * 要求するアドレスのリストは`robstride_constant.h`内の
 * `ROBSTRIDE_READABLE_ADDRESS_LIST`マクロで定義されています。
 * @param device_info Robstrideデバイス情報構造体へのポインタ
 */
void Robstride_RequestAllParameters(Robstride_DeviceInfo *const device_info, DelayFunction_t f_delay) {
    printf("--- Requesting all parameters for device ID: %d ---\n\r", device_info->device_id);

    // マクロを使用してアドレスの配列を初期化
    const uint16_t addresses[] = ROBSTRIDE_READABLE_ADDRESS_LIST;

    const uint8_t num_addresses = sizeof(addresses) / sizeof(addresses[0]);

    for (uint8_t i = 0; i < num_addresses; i++) {
        Robstride_RequestReadParameter(device_info, addresses[i]);
        f_delay(1); // モーターの応答とCANバスの負荷を考慮して短い遅延を入れる
    }
}
/**
 * @brief Robstride_FeedbackData構造体の内容をすべて表示します。
 * @param fb_data 表示するデータが格納された構造体へのポインタ
 */
/**
 * @brief Robstride_FeedbackData構造体の内容をすべて表示します。
 * ファームウェアのバージョンに応じて表示項目が切り替わります。
 * @param fb_data 表示するデータが格納された構造体へのポインタ
 */
void Robstride_PrintAllParameters(const Robstride_FeedbackData *const fb_data) {
    if (fb_data == NULL) {
        printf("Error: Feedback data pointer is NULL.\n\r");
        return;
    }
    printf("\n\r--- Robstride Parameters (Device ID: %d) ---\n\r", fb_data->device_id);
    printf(" [Real-time Data]\n\r");
    printf("  - Position:         %f\n\r", fb_data->position);
    printf("  - Velocity:         %f\n\r", fb_data->velocity);
    printf("  - Current (IqF):    %f\n\r", fb_data->current);
    printf("  - Torque (Type 2):  %f\n\r", fb_data->torque);
    printf("  - Temperature:      %d C\n\r", fb_data->temperature);
    printf("  - Bus Voltage:      %f V\n\r", fb_data->vbus);
    printf("\n\r [Control & Limit Settings]\n\r");
    printf("  - Run Mode:         %u\n\r", fb_data->run_mode);
    printf("  - Iq Ref:           %f A\n\r", fb_data->iq_ref);
    printf("  - Speed Ref:        %f rad/s\n\r", fb_data->spd_ref);
    printf("  - Position Ref:     %f rad\n\r", fb_data->loc_ref);
    printf("  - Limit Torque:     %f Nm\n\r", fb_data->limit_torque);
    printf("  - Limit Speed:      %f rad/s\n\r", fb_data->limit_spd);
    printf("  - Limit Current:    %f A\n\r", fb_data->limit_cur);
    printf("\n\r [PID Gains]\n\r");
    printf("  - Current Kp:       %f\n\r", fb_data->cur_kp);
    printf("  - Current Ki:       %f\n\r", fb_data->cur_ki);
    printf("  - Current Filt Gain:%f\n\r", fb_data->cur_filt_gain);
    printf("  - Position Kp:      %f\n\r", fb_data->loc_kp);
    printf("  - Speed Kp:         %f\n\r", fb_data->spd_kp);
    printf("  - Speed Ki:         %f\n\r", fb_data->spd_ki);
    printf("  - Speed Filt Gain:  %f\n\r", fb_data->spd_filt_gain);
    printf("\n\r [Profile Settings (PP Mode)]\n\r");
#ifndef USE_OLD_FIRMWARE_ADDRESSES
    // 新しいファームウェアのみに存在するパラメータ
    printf("  - Acceleration:     %f rad/s^2\n\r", fb_data->acc_rad);
#endif
    printf("  - Max Velocity:     %f rad/s\n\r", fb_data->vel_max_pp);
    printf("  - Acceleration Set: %f rad/s^2\n\r", fb_data->acc_set_pp);
    printf("\n\r [System Settings]\n\r");
    printf("  - Active Report Time:%u\n\r", fb_data->epscan_time);
    printf("  - CAN Timeout:      %lu\n\r", fb_data->can_timeout);
#ifndef USE_OLD_FIRMWARE_ADDRESSES
    // 新しいファームウェアのみに存在するパラメータ
    printf("  - Zero Flag (zero_sta): %u\n\r", fb_data->zero_sta);
    printf("  - Add Offset:           %f\n\r", fb_data->add_offset);
#endif
    printf("------------------------------------------\n\r\n\r");
}

void Robstride_Debug_Check_All_Parameters(Robstride_DeviceInfo *const device_info, DelayFunction_t f_delay) {
    Robstride_RequestAllParameters(device_info, f_delay);
    f_delay(100); // 全てのパラメータが更新されるまで少し待機
    Robstride_FeedbackData data = Read_Robstride_FeedbackData(device_info);
    Robstride_PrintAllParameters(&data);
}

static HAL_StatusTypeDef robstride_send_current(Robstride_DeviceInfo *const device_info,
                                                const float current)
{
    if (device_info == NULL) {
        return HAL_ERROR;
    }

    const float limited_current = robstride_clamp_current(device_info, current);

    device_info->ctrl_param._req_value = limited_current;
    float wire_current = limited_current;
    if (device_info->ctrl_param.rotation == ROBSTRIDE_ROT_CW) {
        wire_current *= -1.0f;
    }
    return Robstride_WriteFloatData(device_info, ADDR_IQ_REF, wire_current);
}

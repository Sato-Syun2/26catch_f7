#include "can_devices.h"

#include <math.h>
#include <stdio.h>

#include "CAN_Robstride_Def.h"
#include "robstride_constant.h"

/*
 * The CAN devices are prepared before the scheduler starts.  The blocking
 * connection wait and the remaining motor setup run in a FreeRTOS task so
 * that micro-ROS can publish the discovery state while a motor is missing.
 */
static volatile bool can_devices_prepared = false;
static volatile bool can_devices_initialized = false;

/* 実験用の初期値。同定後は各デバイスのvelocity_dobを置き換える。 */
static void configure_robomas_velocity_dob(
    RoboMas_Actuator_VelocityDob_Parameters *const parameters,
    const float velocity_limit,
    const float current_limit,
    const float velocity_unit_to_rad_s,
    const bool velocity_limit_enable,
    const bool current_limit_enable,
    const bool torque_limit_enable)
{
    parameters->J = 1.0e-3f;
    parameters->d = 1.0e-2f;
    parameters->K_tau = 1.0f;
    parameters->dob_bandwidth = 20.0f;
    parameters->velocity_kp = 0.5f;
    parameters->velocity_ki = 1.0f;
    parameters->velocity_kd = 0.0f;
    parameters->reference_alpha = 10.0f;
    parameters->velocity_limit = velocity_limit;
    parameters->current_limit = current_limit;
    /* 初期K_tau=1.0f [Nm/A]に基づく有効なトルク値を保持する。適用可否はflagで決める。 */
    parameters->torque_limit = current_limit * parameters->K_tau;
    parameters->velocity_limit_enable = velocity_limit_enable;
    parameters->current_limit_enable = current_limit_enable;
    parameters->torque_limit_enable = torque_limit_enable;
    parameters->velocity_unit_to_rad_s = velocity_unit_to_rad_s;
    parameters->control_period = 0.002f;
}

static void configure_robstride_velocity_dob(
    Robstride_Actuator_VelocityDob_Parameters *const parameters,
    const float velocity_limit,
    const float current_limit,
    const float velocity_unit_to_rad_s,
    const bool velocity_limit_enable,
    const bool current_limit_enable,
    const bool torque_limit_enable)
{
    parameters->J = 1.0e-3f;
    parameters->d = 1.0e-2f;
    parameters->K_tau = 1.0f;
    parameters->dob_bandwidth = 20.0f;
    parameters->velocity_kp = 0.5f;
    parameters->velocity_ki = 1.0f;
    parameters->velocity_kd = 0.0f;
    parameters->reference_alpha = 10.0f;
    parameters->velocity_limit = velocity_limit;
    parameters->current_limit = current_limit;
    /* 初期K_tau=1.0f [Nm/A]に基づく有効なトルク値を保持する。適用可否はflagで決める。 */
    parameters->torque_limit = current_limit * parameters->K_tau;
    parameters->velocity_limit_enable = velocity_limit_enable;
    parameters->current_limit_enable = current_limit_enable;
    parameters->torque_limit_enable = torque_limit_enable;
    parameters->velocity_unit_to_rad_s = velocity_unit_to_rad_s;
    parameters->control_period = 0.002f;
}

/* FreeRTOS 開始前の初期化処理で使用する待機関数。 */
static float normalize_robstride_startup_position(float position)
{
    float normalized = fmodf(position + 180.0f, 360.0f);

    if (normalized < 0.0f) {
        normalized += 360.0f;
    }

    return normalized - 180.0f;
}

static bool initialize_robstride_position(Robstride_DeviceInfo *device,
                                           float *normalized_position)
{
    const Robstride_FeedbackData feedback = Read_Robstride_FeedbackData(device);

    if ((feedback.get_flag == 0U) || !isfinite(feedback.position)) {
        return false;
    }

    *normalized_position = normalize_robstride_startup_position(feedback.position);

    /* 起動時だけ現在値を正規化し、周回差は位置オフセットとして保持する。 */
    device->ctrl_param.offset_pos += *normalized_position - feedback.position;
    Robstride_fb_init(device);

    return true;
}

/*
 * CAN_Main 用の接続台数設定。
 * 現在は CAN_Main 配下の周辺機器を使用しないため、すべて 0 にする。
 */
const NUM_OF_DEVICES num_of_devices = {
    .servo = 0U,
    .air = 0U,
    .mcmd4 = 0U,
    .dynamixel_kondo = 0U,
    .vl53l0x = 0U,
};

/* Robstride の拡張 CAN プロトコルで使用するマスター ID。 */
const uint8_t master_can_id = 0xFDU;
const uint8_t num_of_robstride = ROBSTRIDE_DEVICE_COUNT;
const uint8_t num_of_c610 = ROBOMAS_C610_COUNT;
const uint8_t num_of_c620 = ROBOMAS_C620_COUNT;
const uint8_t num_of_robomas = ROBOMAS_DEVICE_COUNT;

/* 他タスクから参照するフィードバック・目標値の共有領域。 */
Robstride_FeedbackData robstride_fb[ROBSTRIDE_DEVICE_STORAGE_COUNT];
RoboMas_FeedbackData robomas_fb[ROBOMAS_DEVICE_STORAGE_COUNT];
Robstride_FeedbackData feedback_data[ROBSTRIDE_DEVICE_STORAGE_COUNT];
Robstride_FeedbackData feedback_data_raw[ROBSTRIDE_DEVICE_STORAGE_COUNT];
float feedback_offset[ROBSTRIDE_DEVICE_STORAGE_COUNT];
bool robstride_first_flag = true;
volatile float robstride_target_value[ROBSTRIDE_DEVICE_STORAGE_COUNT];

/*
 * Robstride は CAN3 に接続する。
 * - [0]: Robstride 05 Edu、CAN ID 127（1 台確認時の既定）
 * - [1]: Robstride 02、CAN ID 2
 */
Robstride_DeviceInfo robstride_dev_info_global[ROBSTRIDE_DEVICE_STORAGE_COUNT] = {
#if ROBSTRIDE_DEVICE_COUNT > 0U
    {.device = ROBSTRIDE_DEVICE_0_TYPE, .device_id = ROBSTRIDE_DEVICE_0_ID,
     .master_id = 0xFDU},
#endif
#if ROBSTRIDE_DEVICE_COUNT > 1U
    {.device = ROBSTRIDE_DEVICE_1_TYPE, .device_id = ROBSTRIDE_DEVICE_1_ID,
     .master_id = 0xFDU},
#endif
};

/*
 * RoboMaster は CAN2 に接続する。
 * - [0], [1]: C610、CAN ID 1, 4
 * - [2], [3]: C620、CAN ID 4, 3
 */
RoboMas_DeviceInfo robomas_dev_info_global[ROBOMAS_DEVICE_STORAGE_COUNT] = {
#if ROBOMAS_C610_COUNT > 0U
    [0] = {.device_type = ROBOMASTER_C610, .device_id = ROBOMAS_C610_DEVICE_0_ID},
#endif
#if ROBOMAS_C610_COUNT > 1U
    [1] = {.device_type = ROBOMASTER_C610, .device_id = ROBOMAS_C610_DEVICE_1_ID},
#endif
#if ROBOMAS_C620_COUNT > 0U
    [ROBOMAS_C610_COUNT] = {.device_type = ROBOMASTER_C620,
                            .device_id = ROBOMAS_C620_DEVICE_0_ID},
#endif
#if ROBOMAS_C620_COUNT > 1U
    [ROBOMAS_C610_COUNT + 1U] = {.device_type = ROBOMASTER_C620,
                                 .device_id = ROBOMAS_C620_DEVICE_1_ID},
#endif
};

/*
 * RoboMaster 共通設定。
 * 各デバイス固有の PID、回転方向、減速比は下の個別関数で上書きする。
 */
#if ROBOMAS_DEVICE_COUNT > 0U
static void configure_robomas_common(RoboMas_DeviceInfo *device)
{
    RoboMas_Ctrl_StructTypedef *ctrl = &device->ctrl_param;

    ctrl->use_internal_offset = ROBOMAS_USE_OFFSET_POS_INTERNAL;
    ctrl->ctrl_type = ROBOMAS_CTRL_POS_AW;
    /* Imported from the Robomaster calibration branch. */
    ctrl->current_limit = ROBOMAS_LIMIT_ENABLE;
    ctrl->velocity_limit = ROBOMAS_LIMIT_ENABLE;
}

/* C610 #1（CAN ID 4）の設定。PID はここで個別に変更する。 */
#if ROBOMAS_C610_COUNT > 0U
static void configure_c610_1(void)
{
    RoboMas_Ctrl_StructTypedef *ctrl = &robomas_dev_info_global[0].ctrl_param;
    configure_robomas_common(&robomas_dev_info_global[0]);

    ctrl->rotation = ROBOMAS_ROT_CW;
    ctrl->use_internal_offset = ROBOMAS_USE_OFFSET_POS_CALIB;
    /* Current ID1: 28*pi mm per output-shaft revolution, with M2006 36:1. */
    ctrl->quant_per_rot = 99.0f / 36.0f;
    ctrl->current_limit_size = 2.0f;
    ctrl->velocity_limit_size = 10.0f;
    ctrl->pid_vel.kp = 2.0f;
    ctrl->pid_vel.ki = 1.5f;
    ctrl->pid_vel.kd = 0.0f;
    ctrl->pid_vel.kff = 0.0f;
    ctrl->pid_pos.kp = 3.0f;
    ctrl->pid_pos.ki = 0.0f;
    ctrl->pid_pos.kd = 0.0f;
    ctrl->pid_pos.kff = 0.0f;
    configure_robomas_velocity_dob(&ctrl->velocity_dob,
                                   ctrl->velocity_limit_size,
                                   ctrl->current_limit_size,
                                   1.0f,
                                   ctrl->velocity_limit == ROBOMAS_LIMIT_ENABLE,
                                   ctrl->current_limit == ROBOMAS_LIMIT_ENABLE,
                                   false);
}
#endif

/* C610 #2（CAN ID 1）の設定。必要に応じて #1 と異なる値を設定する。 */
#if ROBOMAS_C610_COUNT > 1U
static void configure_c610_2(void)
{
    RoboMas_Ctrl_StructTypedef *ctrl = &robomas_dev_info_global[1].ctrl_param;
    configure_robomas_common(&robomas_dev_info_global[1]);

    ctrl->rotation = ROBOMAS_ROT_ACW;
    ctrl->use_internal_offset = ROBOMAS_USE_OFFSET_POS_CALIB;
    ctrl->quant_per_rot = 28.0f * 3.14159265359f / 36.0f* 2.0f;
    ctrl->current_limit_size = 2.0f;
    ctrl->velocity_limit_size = 10.0f;
    ctrl->pid_vel.kp = 2.0f;
    ctrl->pid_vel.ki = 4.0f;
    ctrl->pid_vel.kd = 0.0f;
    ctrl->pid_vel.kff = 0.0f;
    ctrl->pid_pos.kp = 4.0f;
    ctrl->pid_pos.ki = 0.0f;
    ctrl->pid_pos.kd = 0.0f;
    ctrl->pid_pos.kff = 0.0f;
    configure_robomas_velocity_dob(&ctrl->velocity_dob,
                                   ctrl->velocity_limit_size,
                                   ctrl->current_limit_size,
                                   1.0f,
                                   ctrl->velocity_limit == ROBOMAS_LIMIT_ENABLE,
                                   ctrl->current_limit == ROBOMAS_LIMIT_ENABLE,
                                   false);
}
#endif

/* C620 #1（CAN ID 3）の設定。 */
#if ROBOMAS_C620_COUNT > 0U
static void configure_c620_1(void)
{
    RoboMas_Ctrl_StructTypedef *ctrl =
        &robomas_dev_info_global[ROBOMAS_C610_COUNT].ctrl_param;
    configure_robomas_common(&robomas_dev_info_global[ROBOMAS_C610_COUNT]);

    ctrl->rotation = ROBOMAS_ROT_ACW;
    ctrl->quant_per_rot = 2.0f * 3.14159265359f / 36.0f;
    ctrl->current_limit_size = 2.0f;
    ctrl->velocity_limit_size = 10.0f;
    ctrl->pid_vel.kp = 2.0f;
    ctrl->pid_vel.ki = 4.0f;
    ctrl->pid_vel.kd = 0.0f;
    ctrl->pid_vel.kff = 0.0f;
    ctrl->pid_pos.kp = 4.0f;
    ctrl->pid_pos.ki = 0.0f;
    ctrl->pid_pos.kd = 0.0f;
    ctrl->pid_pos.kff = 0.0f;
    configure_robomas_velocity_dob(&ctrl->velocity_dob,
                                   ctrl->velocity_limit_size,
                                   ctrl->current_limit_size,
                                   1.0f,
                                   ctrl->velocity_limit == ROBOMAS_LIMIT_ENABLE,
                                   ctrl->current_limit == ROBOMAS_LIMIT_ENABLE,
                                   false);
}
#endif

/* C620 #2（CAN ID 4）の設定。 */
#if ROBOMAS_C620_COUNT > 1U
static void configure_c620_2(void)
{
    RoboMas_Ctrl_StructTypedef *ctrl =
        &robomas_dev_info_global[ROBOMAS_C610_COUNT + 1U].ctrl_param;
    configure_robomas_common(&robomas_dev_info_global[ROBOMAS_C610_COUNT + 1U]);

    ctrl->rotation = ROBOMAS_ROT_ACW;
    ctrl->quant_per_rot = 2.0f * 3.14159265359f / 36.0f;
    ctrl->current_limit_size = 2.0f;
    ctrl->velocity_limit_size = 10.0f;
    ctrl->pid_vel.kp = 2.0f;
    ctrl->pid_vel.ki = 4.0f;
    ctrl->pid_vel.kd = 0.0f;
    ctrl->pid_vel.kff = 0.0f;
    ctrl->pid_pos.kp = 4.0f;
    ctrl->pid_pos.ki = 0.0f;
    ctrl->pid_pos.kd = 0.0f;
    ctrl->pid_pos.kff = 0.0f;
    configure_robomas_velocity_dob(&ctrl->velocity_dob,
                                   ctrl->velocity_limit_size,
                                   ctrl->current_limit_size,
                                   1.0f,
                                   ctrl->velocity_limit == ROBOMAS_LIMIT_ENABLE,
                                   ctrl->current_limit == ROBOMAS_LIMIT_ENABLE,
                                   false);
}
#endif

#endif

#if ROBSTRIDE_DEVICE_COUNT > 0U
static void configure_robstride_common(Robstride_DeviceInfo *device)
{
    Robstride_Ctrl_StructTypedef *ctrl = &device->ctrl_param;

    ctrl->use_internal_offset = ROBSTRIDE_USE_OFFSET_POS_INTERNAL;
    ctrl->ctrl_type = ROBSTRIDE_CTRL_VEL_DOB;
    ctrl->velocity_limit = ROBSTRIDE_VELOCITY_LIMIT_ENABLE;
    ctrl->current_limit = ROBSTRIDE_CURRENT_LIMIT_ENABLE;
    ctrl->torque_limit = ROBSTRIDE_TORQUE_LIMIT_DISABLE;
    ctrl->rotation = ROBSTRIDE_ROT_CW;
    ctrl->velocity_limit_size = 1.57079632679f;
    ctrl->current_limit_size = 2.0f;
    ctrl->torque_limit_size = 17.0f;
    ctrl->quant_per_rot = 360.0f / (2.0f * 3.14159265359f);
    configure_robstride_velocity_dob(&ctrl->velocity_dob,
                                     ctrl->velocity_limit_size,
                                     ctrl->current_limit_size,
                                     1.0f / ctrl->quant_per_rot,
                                     ctrl->velocity_limit ==
                                         ROBSTRIDE_VELOCITY_LIMIT_ENABLE,
                                     ctrl->current_limit ==
                                         ROBSTRIDE_CURRENT_LIMIT_ENABLE,
                                     ctrl->torque_limit ==
                                         ROBSTRIDE_TORQUE_LIMIT_ENABLE);
}

/* Robstride スロット 0 の設定。 */
#if ROBSTRIDE_DEVICE_COUNT > 0U
static void configure_robstride_0(void)
{
    Robstride_Ctrl_StructTypedef *ctrl = &robstride_dev_info_global[0].ctrl_param;
    configure_robstride_common(&robstride_dev_info_global[0]);

    /* ID2（根本）は速度制限を無効化し、通常確認用に2Aへ設定する。 */
    ctrl->velocity_limit = ROBSTRIDE_VELOCITY_LIMIT_DISABLE;
    ctrl->velocity_dob.velocity_limit = 44.0f;
    ctrl->velocity_dob.velocity_limit_enable = false;
    ctrl->velocity_limit_size = 44.0f; /* Robstride_02の速度上限 */
    ctrl->current_limit = ROBSTRIDE_CURRENT_LIMIT_ENABLE;
    ctrl->current_limit_size = 2.0f; /* 通常確認用の電流上限 */
    ctrl->offset_pos = 8.0f;
    ctrl->pid.kp_pos = 7.0f;
    ctrl->pid.kp_vel = 6.0f;
    ctrl->pid.ki_vel = 0.02f;
    ctrl->pid.filter_vel = 0.06f;
    ctrl->pid.kp_cur = 0.05f;
    ctrl->pid.ki_cur = 0.05f;
    ctrl->pid.filter_cur = 0.06f;
}
#endif

/* Robstride スロット 1 の設定。 */
#if ROBSTRIDE_DEVICE_COUNT > 1U
static void configure_robstride_1(void)
{
    Robstride_Ctrl_StructTypedef *ctrl = &robstride_dev_info_global[1].ctrl_param;
    configure_robstride_common(&robstride_dev_info_global[1]);

    ctrl->offset_pos = 67.0f;
    ctrl->pid.kp_pos = 7.0f;
    ctrl->pid.kp_vel = 6.0f;
    ctrl->pid.ki_vel = 0.02f;
    ctrl->pid.filter_vel = 0.06f;
    ctrl->pid.kp_cur = 0.05f;
    ctrl->pid.ki_cur = 0.05f;
    ctrl->pid.filter_cur = 0.06f;
}
#endif

#endif

bool CanDevices_IsPrepared(void)
{
    return can_devices_prepared;
}

bool CanDevices_IsInitialized(void)
{
    return can_devices_initialized;
}

Robstride_FeedbackData CanDevices_GetRobstrideFeedback(const uint32_t index)
{
    if (index >= ROBSTRIDE_DEVICE_COUNT) {
        return (Robstride_FeedbackData){0};
    }

    if (can_devices_prepared) {
        /* During WaitForConnect(), this is the data updated by CAN RX ISR. */
        return Read_Robstride_FeedbackData(&robstride_dev_info_global[index]);
    }

    return feedback_data[index];
}

void CanDevices_InitBeforeWait(CAN_HandleTypeDef *robomas_can,
                               CAN_HandleTypeDef *robstride_can,
                               DelayFunction_t delay_function)
{
    (void)delay_function;
    can_devices_prepared = false;
    can_devices_initialized = false;

    /* CAN2 上の RoboMaster を初期化してから、個別設定を反映する。 */
    Init_RoboMas_CAN_System(robomas_can);
    RoboMas_Init(robomas_dev_info_global, ROBOMAS_DEVICE_COUNT);
#if ROBOMAS_C610_COUNT > 0U
    configure_c610_1();
#endif
#if ROBOMAS_C610_COUNT > 1U
    configure_c610_2();
#endif
#if ROBOMAS_C620_COUNT > 0U
    configure_c620_1();
#endif
#if ROBOMAS_C620_COUNT > 1U
    configure_c620_2();
#endif

    /*
     * RoboMaster feedback is received asynchronously from CAN2.  Do not
     * block here waiting for it and do not enable a motor at boot; the
     * RobomasTask publishes feedback while the ROS service controls enable.
     */

    /* CAN3 上の Robstride を初期化してから、個別設定を反映する。 */
    Init_Robstride_CAN_System(robstride_can);
    Robstride_Init(robstride_dev_info_global, ROBSTRIDE_DEVICE_COUNT);
#if ROBSTRIDE_DEVICE_COUNT > 0U
    robstride_dev_info_global[0].phcan = robstride_can;
    configure_robstride_0();
    Robstride_fb_init(&robstride_dev_info_global[0]);
#endif
#if ROBSTRIDE_DEVICE_COUNT > 1U
    robstride_dev_info_global[1].phcan = robstride_can;
    configure_robstride_1();
    Robstride_fb_init(&robstride_dev_info_global[1]);
#endif

    can_devices_prepared = true;
}

void CanDevices_InitAfterWait(DelayFunction_t delay_function)
{
    /* 接続確認後、停止状態で全パラメータを反映してから制御を有効化する。 */
    Robstride_WaitForConnect(robstride_dev_info_global,
                             ROBSTRIDE_DEVICE_COUNT,
                             delay_function);
    for (uint8_t i = 0U; i < ROBSTRIDE_DEVICE_COUNT; ++i) {
        Robstride_DeviceInfo *const device = &robstride_dev_info_global[i];
        float initial_position;

        Robstride_ControlDisable(device, delay_function);
        Robstride_SetPIDParams(device, delay_function);
        /*
         * パラメータ読出し応答が来ない個体でも、ここで起動を止めない。
         * 設定値は個別設定済みかつ範囲内なので、書込みのみで反映する。
         */
        Robstride_WriteFloatData(device, ADDR_LIMIT_SPEED,
                                 device->ctrl_param.velocity_limit_size);
        delay_function(10U);
        Robstride_WriteFloatData(device, ADDR_LIMIT_CURRENT,
                                 device->ctrl_param.current_limit_size);
        delay_function(10U);
        /* 起動時に書き込んだ速度・電流リミットをモーターから読み返す。 */
        Robstride_RequestReadParameter(device, ADDR_LIMIT_SPEED);
        delay_function(10U);
        Robstride_RequestReadParameter(device, ADDR_LIMIT_CURRENT);
        delay_function(10U);
        {
            const Robstride_FeedbackData applied_limits =
                Read_Robstride_FeedbackData(device);
            printf("[Robstride] ID %u applied limits: speed=%.6f rad/s, current=%.6f A\r\n",
                   (unsigned int)device->device_id,
                   (double)applied_limits.limit_spd,
                   (double)applied_limits.limit_cur);
        }
        Robstride_SetTorqueLimit(device);
        delay_function(10U);

        if (!initialize_robstride_position(device, &initial_position)) {
            continue;
        }

        /* micro-ROSの指令値と同じ度数法で、起動時の現在値を目標にする。 */
        const float initial_target =
            device->ctrl_param.ctrl_type == ROBSTRIDE_CTRL_VEL_DOB
                ? 0.0f
                : initial_position;
        robstride_target_value[i] = initial_target;
        Robstride_SetTarget(device, initial_target);
        Robstride_SetControl(device, device->ctrl_param.ctrl_type, delay_function);
    }

    can_devices_initialized = true;
}

void CanDevices_Init(CAN_HandleTypeDef *robomas_can,
                     CAN_HandleTypeDef *robstride_can,
                     DelayFunction_t delay_function)
{
    /* Preserve the original synchronous API for other callers. */
    CanDevices_InitBeforeWait(robomas_can, robstride_can, delay_function);
    CanDevices_InitAfterWait(delay_function);
}

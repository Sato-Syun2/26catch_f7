#include "can_devices.h"

#include <math.h>
#include <stdio.h>

#include "CAN_Robstride_Def.h"
#include "robstride_constant.h"

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
 * - [0], [1]: C610、CAN ID 1, 2
 * - [2], [3]: C620、CAN ID 3, 4
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
    ctrl->ctrl_type = ROBOMAS_CTRL_POS;
    ctrl->current_limit = ROBOMAS_LIMIT_DISABLE;
    ctrl->velocity_limit = ROBOMAS_LIMIT_DISABLE;
}

/* C610 #1（CAN ID 1）の設定。PID はここで個別に変更する。 */
static void configure_c610_1(void)
{
    RoboMas_Ctrl_StructTypedef *ctrl = &robomas_dev_info_global[0].ctrl_param;
    configure_robomas_common(&robomas_dev_info_global[0]);

    ctrl->rotation = ROBOMAS_ROT_CW;
    ctrl->quant_per_rot = 2.0f * 3.14159265359f / 36.0f * 2.0f;
    ctrl->pid_vel.kp = 2.0f;
    ctrl->pid_vel.ki = 4.0f;
    ctrl->pid_vel.kd = 0.0f;
    ctrl->pid_vel.kff = 0.0f;
    ctrl->pid_pos.kp = 4.0f;
    ctrl->pid_pos.ki = 0.0f;
    ctrl->pid_pos.kd = 0.0f;
    ctrl->pid_pos.kff = 0.0f;
}

/* C610 #2（CAN ID 2）の設定。必要に応じて #1 と異なる値を設定する。 */
static void configure_c610_2(void)
{
    RoboMas_Ctrl_StructTypedef *ctrl = &robomas_dev_info_global[1].ctrl_param;
    configure_robomas_common(&robomas_dev_info_global[1]);

    ctrl->rotation = ROBOMAS_ROT_ACW;
    ctrl->quant_per_rot = 2.0f * 3.14159265359f / 36.0f;
    ctrl->pid_vel.kp = 2.0f;
    ctrl->pid_vel.ki = 4.0f;
    ctrl->pid_vel.kd = 0.0f;
    ctrl->pid_vel.kff = 0.0f;
    ctrl->pid_pos.kp = 4.0f;
    ctrl->pid_pos.ki = 0.0f;
    ctrl->pid_pos.kd = 0.0f;
    ctrl->pid_pos.kff = 0.0f;
}

/* C620 #1（CAN ID 3）の設定。 */
static void configure_c620_1(void)
{
    RoboMas_Ctrl_StructTypedef *ctrl =
        &robomas_dev_info_global[ROBOMAS_C610_COUNT].ctrl_param;
    configure_robomas_common(&robomas_dev_info_global[ROBOMAS_C610_COUNT]);

    ctrl->rotation = ROBOMAS_ROT_ACW;
    ctrl->quant_per_rot = 2.0f * 3.14159265359f / 36.0f;
    ctrl->pid_vel.kp = 2.0f;
    ctrl->pid_vel.ki = 4.0f;
    ctrl->pid_vel.kd = 0.0f;
    ctrl->pid_vel.kff = 0.0f;
    ctrl->pid_pos.kp = 4.0f;
    ctrl->pid_pos.ki = 0.0f;
    ctrl->pid_pos.kd = 0.0f;
    ctrl->pid_pos.kff = 0.0f;
}

/* C620 #2（CAN ID 4）の設定。 */
static void configure_c620_2(void)
{
    RoboMas_Ctrl_StructTypedef *ctrl =
        &robomas_dev_info_global[ROBOMAS_C610_COUNT + 1U].ctrl_param;
    configure_robomas_common(&robomas_dev_info_global[ROBOMAS_C610_COUNT + 1U]);

    ctrl->rotation = ROBOMAS_ROT_ACW;
    ctrl->quant_per_rot = 2.0f * 3.14159265359f / 36.0f;
    ctrl->pid_vel.kp = 2.0f;
    ctrl->pid_vel.ki = 4.0f;
    ctrl->pid_vel.kd = 0.0f;
    ctrl->pid_vel.kff = 0.0f;
    ctrl->pid_pos.kp = 4.0f;
    ctrl->pid_pos.ki = 0.0f;
    ctrl->pid_pos.kd = 0.0f;
    ctrl->pid_pos.kff = 0.0f;
}

#endif

static void configure_robstride_common(Robstride_DeviceInfo *device)
{
    Robstride_Ctrl_StructTypedef *ctrl = &device->ctrl_param;

    ctrl->use_internal_offset = ROBSTRIDE_USE_OFFSET_POS_INTERNAL;
    ctrl->ctrl_type = ROBSTRIDE_CTRL_POS;
    ctrl->velocity_limit = ROBSTRIDE_VELOCITY_LIMIT_ENABLE;
    ctrl->current_limit = ROBSTRIDE_CURRENT_LIMIT_ENABLE;
    ctrl->torque_limit = ROBSTRIDE_TORQUE_LIMIT_DISABLE;
    ctrl->rotation = ROBSTRIDE_ROT_CW;
    ctrl->velocity_limit_size = 1.57079632679f;
    ctrl->current_limit_size = 2.0f;
    ctrl->torque_limit_size = 17.0f;
    ctrl->quant_per_rot = 360.0f / (2.0f * 3.14159265359f);
}

/* Robstride スロット 0 の設定。 */
static void configure_robstride_0(void)
{
    Robstride_Ctrl_StructTypedef *ctrl = &robstride_dev_info_global[0].ctrl_param;
    configure_robstride_common(&robstride_dev_info_global[0]);

    /* ID2（根本）は速度制限を無効化し、通常確認用に2Aへ設定する。 */
    ctrl->velocity_limit = ROBSTRIDE_VELOCITY_LIMIT_DISABLE;
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

/* Robstride スロット 1 の設定。 */
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

void CanDevices_Init(CAN_HandleTypeDef *robomas_can,
                     CAN_HandleTypeDef *robstride_can,
                     DelayFunction_t delay_function)
{
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

    /* 接続を確認してから、RoboMaster の制御を有効化する。 */
    RoboMas_WaitForConnect(robomas_dev_info_global,
                           ROBOMAS_DEVICE_COUNT,
                           delay_function);
    for (uint8_t i = 0U; i < ROBOMAS_DEVICE_COUNT; ++i) {
        RoboMas_SetTarget(&robomas_dev_info_global[i], 0.0f);
        RoboMas_ControlEnable(&robomas_dev_info_global[i]);
    }

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
        /* 起動時に書き込んだ速度・電流リミットをモータから読み返す。 */
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
        robstride_target_value[i] = initial_position;
        Robstride_SetTarget(device, initial_position);
        Robstride_SetControl(device, device->ctrl_param.ctrl_type, delay_function);
    }
}

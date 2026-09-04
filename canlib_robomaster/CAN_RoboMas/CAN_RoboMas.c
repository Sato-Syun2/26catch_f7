//
// Created by emile on 23/07/13.
//

// Includes --------------------------------
#include "CAN_RoboMas_Def.h"
#include "CAN_RoboMas_System.h"
#include "CAN_RoboMas.h"
#include "math.h"
#include "stdio.h"

// Private Function Prototypes --------------------------------
static bool get_switch_state(GPIO_TypeDef* limit_port, uint16_t limit_pin, ROBOMAS_SWITCH_TYPE sw_type);
static float _clip_f_abs(float var, float abs_ref);
static int16_t c610_current_f2int(float current);
static int16_t c620_current_f2int(float current);
static void RoboMas_Ctrl_Struct_init(RoboMas_Ctrl_StructTypedef *ctrl_struct);
static bool robomas_is_position_control(ROBOMAS_CTRL_TYPE ctrl_type);

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
    return ctrl_type == ROBOMAS_CTRL_POS || ctrl_type == ROBOMAS_CTRL_POS_AW;
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

        if (!(dev_info_array[i].ctrl_param._enable_flag) ||
            !(dev_info_array[i].ctrl_param._target_valid)) {
            dev_info_array[i].ctrl_param._req_value = 0.0f;
            continue;
        }

        const RoboMas_FeedbackData fb_data = Get_RoboMas_FeedbackData(&dev_info_array[i]);

        if (dev_info_array[i].ctrl_param._is_calibrating) {
            if (get_switch_state(dev_info_array[i].ctrl_param._limit_port, dev_info_array[i].ctrl_param._limit_pin, dev_info_array[i].ctrl_param._sw_type)) {
                dev_info_array[i].ctrl_param._is_calibrating = false;

                _change_internal_offset_for_calib(&dev_info_array[i]);
                
                RoboMas_ControlDisable(&dev_info_array[i]);
                RoboMas_ChangeControl(&dev_info_array[i], dev_info_array[i].ctrl_param._ctrl_type_before_calib);
                if(robomas_is_position_control(dev_info_array[i].ctrl_param._ctrl_type_before_calib)){
                    RoboMas_SetTarget(&dev_info_array[i], dev_info_array[i].ctrl_param.offset_pos);
                }else{
                    RoboMas_SetTarget(&dev_info_array[i], 0.0f);
                }
                RoboMas_ControlEnable(&dev_info_array[i]);

                // Disable 状態のまま返す
                continue;
            }
        }

        if (dev_info_array[i].ctrl_param.ctrl_type == ROBOMAS_CTRL_CURRENT) {
            t_current = dev_info_array[i].ctrl_param._target_value;
        } else if (dev_info_array[i].ctrl_param.ctrl_type == ROBOMAS_CTRL_VEL_DOB) {
            const float dt = update_freq_hz > 0.0f
                                 ? 1.0f / update_freq_hz
                                 : 0.0f;
            t_current = RoboMas_Actuator_VelocityDob_Update(
                &(dev_info_array[i].ctrl_param.velocity_dob),
                &(dev_info_array[i].ctrl_param.velocity_dob_state),
                dev_info_array[i].ctrl_param._target_value,
                fb_data.velocity,
                dt);
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
                const float t_vel = RoboMas_PID_Ctrl_AW(&(dev_info_array[i].ctrl_param.pid_pos), diff, dev_info_array[i].ctrl_param.velocity_limit == ROBOMAS_LIMIT_ENABLE, dev_info_array[i].ctrl_param.velocity_limit_size, update_freq_hz);
                t_current = RoboMas_PID_Ctrl_AW(&(dev_info_array[i].ctrl_param.pid_vel), t_vel - fb_data.velocity, dev_info_array[i].ctrl_param.current_limit == ROBOMAS_LIMIT_ENABLE, dev_info_array[i].ctrl_param.current_limit_size, update_freq_hz);
                // 位置制御の場合は速度と位置の2重でPID
            }else if(dev_info_array[i].ctrl_param.ctrl_type == ROBOMAS_CTRL_VEL){
                t_current = RoboMas_PID_Ctrl_AW(&(dev_info_array[i].ctrl_param.pid_vel), diff, dev_info_array[i].ctrl_param.current_limit == ROBOMAS_LIMIT_ENABLE, dev_info_array[i].ctrl_param.current_limit_size, update_freq_hz);
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
    if(device_info->ctrl_param.use_internal_offset != ROBOMAS_USE_OFFSET_POS_CALIB) return;

    device_info->ctrl_param._sw_type = sw_type;
    device_info->ctrl_param._limit_port = limit_port;
    device_info->ctrl_param._limit_pin = limit_pin;
    device_info->ctrl_param._ctrl_type_before_calib = device_info->ctrl_param.ctrl_type;

    RoboMas_ControlDisable(device_info);
    RoboMas_ChangeControl(device_info, ROBOMAS_CTRL_VEL);
    RoboMas_SetTarget(device_info, calib_vel);
    RoboMas_ControlEnable(device_info);

    device_info->ctrl_param._is_calibrating = true;
}

void RoboMas_ChangeControl(RoboMas_DeviceInfo *dev_info, ROBOMAS_CTRL_TYPE new_ctrl_type) {
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
    dev_info->ctrl_param._enable_flag = true;
}

void RoboMas_ControlDisable(RoboMas_DeviceInfo *dev_info) {
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

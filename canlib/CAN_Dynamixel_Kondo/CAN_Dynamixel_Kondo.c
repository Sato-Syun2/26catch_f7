/*
 * CAN_Dynamixel.c
 *
 *  Created on: 11 21, 2024
 *      Author: Akitomo KURAKU
 */

// Includes --------------------------------

#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "usart.h"

#include "CAN_Dynamixel_Kondo.h"
#include "string.h"
#include "pid.h"

// Variables --------------------------------

CAN_HandleTypeDef *phcan;
uint8_t node_id;
uint8_t is_initialized = 0;
uint8_t initialize_cmd_received = 0;

static UART_HandleTypeDef *phuart; // RS485 に使用する UART

Dynamixel_Kondo_Ctrl_Typedef dynamixel_kondo_ctrl_global[8];

static DYNAMIXEL_OR_KONDO dkboard_type = DKBOARD_UNINITIALIZED; // dynamixel 基板として働くか kondo 基板として働くか
static bool is_rs485_system_initialized = false;                // RS485 システム（すなわち、dma_serial）が既に初期化されたか

// Private Function Prototypes --------------------------------

static void CANDynamixel_SetRotDir(Dynamixel_Kondo_Ctrl_Typedef *can_dynamixel_kondo, Dynamixel_DIR dir);
static bool CANDynamixel_Kondo_Change_Status(Dynamixel_Kondo_Ctrl_Typedef *can_dynamixel_kondo, Dynamixel_Kondo_Status next_status); // 状態遷移可能か判定し, 可能なら遷移する. 状態遷移できた場合はtrueをできなかった場合はfalseを返す. Dynamixel_Kondo基板の状態遷移は全てこの関数を用いて行われる

static Dynamixel_Model_Number _CANDynamixel_getModelNumber(Dynamixel_MODEL model);
static void _CANDynamixel_Kondo_Enable(Dynamixel_Kondo_Ctrl_Typedef *can_dynamixel_kondo);
static void _CANDynamixel_Kondo_Disable(Dynamixel_Kondo_Ctrl_Typedef *can_dynamixel_kondo);
static uint8_t _GetID(void);
static bool _is_usb_connected(void); // PC⇔Dynamixel_Kondo基板のusbが接続されているかどうか（「接続されている場合はF3からは通信しない」という処理を各関数内で行う）

static char *_string_dynamixel_kondo_status(Dynamixel_Kondo_Status status);
static char *_string_dynamixel_kondo_fb_type(Dynamixel_Kondo_FB_TYPE fb_type);
static char *_string_kondo_traj(Kondo_TRAJECTORY_TYPE traj_type);

static float clip_abs_float(float val, float limit); // abs(val) <= limit になるようにして値を返す。limit > 0.0f でない場合は 0.0f を返す。val が NAN なら NAN を返す。

// Functions --------------------------------

HAL_StatusTypeDef SendBytes(CAN_HandleTypeDef *hcan, uint32_t ExtId, uint8_t *bytes, uint32_t size) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;

    phcan = hcan;

    TxHeader.ExtId = ExtId;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_EXT;
    TxHeader.DLC = size;
    TxHeader.TransmitGlobalTime = DISABLE;

    uint32_t quotient = size / 8;
    uint32_t remainder = size - (8 * quotient);

    HAL_StatusTypeDef ret;

    for (uint8_t i = 0; i < quotient; i++) {
        HAL_Delay(2); // FIFOの空きを見るべき
        ret = HAL_CAN_AddTxMessage(hcan, &TxHeader, bytes + i * 8, &TxMailbox);
        if (ret != HAL_OK) {
            Error_Handler();
            return ret;
        } else {
        }
    }

    TxHeader.DLC = remainder;
    if (TxHeader.DLC != 0) {
        HAL_Delay(2); // FIFOの空きを見るべき
        ret = HAL_CAN_AddTxMessage(hcan, &TxHeader, bytes + quotient * 8, &TxMailbox);
        if (ret != HAL_OK) {
            Error_Handler();
            return ret;
        } else {
        }
    }
    return HAL_OK;
}

void InitCANDynamixel_Kondo(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart) {
    phuart = huart;

    node_id = _GetID();               // node_idを取得(基板番号)
    for (uint8_t i = 0; i < 8; ++i) { // モーターのパラメータ初期化
        // device setting

        dynamixel_kondo_ctrl_global[i].dynamixel_or_kondo = DKBOARD_UNINITIALIZED;

        dynamixel_kondo_ctrl_global[i].dynamixel_param.device.device_num = i;
        dynamixel_kondo_ctrl_global[i].dynamixel_param.device.node_id = node_id;
        dynamixel_kondo_ctrl_global[i].dynamixel_param.device.node_type = NODE_DYNAMIXEL_KONDO;

        dynamixel_kondo_ctrl_global[i].kondo_param.device.device_num = i;
        dynamixel_kondo_ctrl_global[i].kondo_param.device.node_id = node_id;
        dynamixel_kondo_ctrl_global[i].kondo_param.device.node_type = NODE_DYNAMIXEL_KONDO;

        dynamixel_kondo_ctrl_global[i]._last_target_pos = 1.0e32f;
        dynamixel_kondo_ctrl_global[i].target_value = 0.0f;
        dynamixel_kondo_ctrl_global[i].target_time = 0.0f;

        dynamixel_kondo_ctrl_global[i].kondo_param.ctrl_param.velocity_limit = 0.6f; // [rps]
        dynamixel_kondo_ctrl_global[i].kondo_param.ctrl_param.velocity_limit = 0.6f; // [rps]
        dynamixel_kondo_ctrl_global[i].kondo_param.ctrl_param.integral_limit = 0.1f; // [rot s]

        // status setting
        dynamixel_kondo_ctrl_global[i].status = DYNAMIXEL_KONDO_STATUS_INIT;
        dynamixel_kondo_ctrl_global[i]._is_initialized = 0;
        dynamixel_kondo_ctrl_global[i]._start_change_ctrl = 0;
        dynamixel_kondo_ctrl_global[i]._finish_change_ctrl = 0; // 0
        dynamixel_kondo_ctrl_global[i].need_dynamixel_kondo_initialize = false;
        dynamixel_kondo_ctrl_global[i].need_dynamixel_kondo_enable = false;
        dynamixel_kondo_ctrl_global[i].need_dynamixel_kondo_disable = false;
        dynamixel_kondo_ctrl_global[i].kondo_extended_position_offset = NAN;
    }

    // CAN初期化
    CAN_FilterTypeDef sFilterConfig;
    printf("InitCANDynamixel_Kondo start\n");

    // ここからCANのフィルター設定
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // IDMASKモード
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 32bitモード
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_DYNAMIXEL_KONDO, node_id, 0, DYNAMIXEL_KONDO_CMD_INIT1) >> 13;
    sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0b111, 0b111, 0, 0b11111) >> 13;
    sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_DYNAMIXEL_KONDO, node_id, 0, DYNAMIXEL_KONDO_CMD_INIT1) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0b111, 0b111, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; // Fifo0に受信

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) // フィルターを登録
    {
        /* Filter configuration Error */
        Error_Handler();
    }

    sFilterConfig.FilterBank = 1;
    sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_DYNAMIXEL_KONDO, node_id, 0, DYNAMIXEL_KONDO_CMD_INIT2) >> 13;
    sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0b111, 0b111, 0, 0b11111) >> 13;
    sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_DYNAMIXEL_KONDO, node_id, 0, DYNAMIXEL_KONDO_CMD_INIT2) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0b111, 0b111, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) {
        /* Filter configuration Error */
        Error_Handler();
    }

    sFilterConfig.FilterBank = 3;
    sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_DYNAMIXEL_KONDO, node_id, 0, DYNAMIXEL_KONDO_CMD_CHANGE_CTRL1) >> 13;
    sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0b111, 0b111, 0, 0b11111) >> 13;
    sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_DYNAMIXEL_KONDO, node_id, 0, DYNAMIXEL_KONDO_CMD_CHANGE_CTRL1) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0b111, 0b111, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) {
        /* Filter configuration Error */
        Error_Handler();
    }

    sFilterConfig.FilterBank = 4;
    sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_DYNAMIXEL_KONDO, node_id, 0, DYNAMIXEL_KONDO_CMD_CHANGE_CTRL2) >> 13;
    sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0b111, 0b111, 0, 0b11111) >> 13;
    sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_DYNAMIXEL_KONDO, node_id, 0, DYNAMIXEL_KONDO_CMD_CHANGE_CTRL2) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0b111, 0b111, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) {
        /* Filter configuration Error */
        Error_Handler();
    }

    sFilterConfig.FilterBank = 5;
    sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_DYNAMIXEL_KONDO, node_id, 0, DYNAMIXEL_KONDO_CMD_CHANGE_CTRL3) >> 13;
    sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0b111, 0b111, 0, 0b11111) >> 13;
    sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_DYNAMIXEL_KONDO, node_id, 0, DYNAMIXEL_KONDO_CMD_CHANGE_CTRL3) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0b111, 0b111, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) {
        /* Filter configuration Error */
        Error_Handler();
    }

    sFilterConfig.FilterBank = 6;
    sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_DYNAMIXEL_KONDO, node_id, 0, DYNAMIXEL_KONDO_CMD_CHANGE_CTRL4) >> 13;
    sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0b111, 0b111, 0, 0b11111) >> 13;
    sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_DYNAMIXEL_KONDO, node_id, 0, DYNAMIXEL_KONDO_CMD_CHANGE_CTRL4) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0b111, 0b111, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) {
        /* Filter configuration Error */
        Error_Handler();
    }

    sFilterConfig.FilterBank = 7;
    sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_DYNAMIXEL_KONDO, node_id, 0, DYNAMIXEL_KONDO_CMD_ENABLE) >> 13;
    sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0b111, 0b111, 0, 0b11111) >> 13;
    sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_DYNAMIXEL_KONDO, node_id, 0, DYNAMIXEL_KONDO_CMD_ENABLE) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0b111, 0b111, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) {
        /* Filter configuration Error */
        Error_Handler();
    }

    sFilterConfig.FilterBank = 8;
    sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_DYNAMIXEL_KONDO, node_id, 0, DYNAMIXEL_KONDO_CMD_DISABLE) >> 13;
    sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0b111, 0b111, 0, 0b11111) >> 13;
    sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_DYNAMIXEL_KONDO, node_id, 0, DYNAMIXEL_KONDO_CMD_DISABLE) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0b111, 0b111, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) {
        /* Filter configuration Error */
        Error_Handler();
    }

    sFilterConfig.FilterBank = 9;
    sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_DYNAMIXEL_KONDO, node_id, 0, DYNAMIXEL_KONDO_CMD_SET_TARGET) >> 13;
    sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0b111, 0b111, 0, 0b11111) >> 13;
    sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_DYNAMIXEL_KONDO, node_id, 0, DYNAMIXEL_KONDO_CMD_SET_TARGET) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0b111, 0b111, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) {
        /* Filter configuration Error */
        Error_Handler();
    }

    sFilterConfig.FilterBank = 10;
    sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_DYNAMIXEL_KONDO, node_id, 0, DYNAMIXEL_KONDO_CMD_CHANGE_CTRL5) >> 13;
    sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0b111, 0b111, 0, 0b11111) >> 13;
    sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_DYNAMIXEL_KONDO, node_id, 0, DYNAMIXEL_KONDO_CMD_CHANGE_CTRL5) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0b111, 0b111, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) {
        /* Filter configuration Error */
        Error_Handler();
    }
    // ここまでフィルター設定

    printf("CAN start\n");
    if (HAL_CAN_Start(hcan) != HAL_OK) {
        /* Start Error */
        printf("Start Error\n");
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        printf("FIFO0 CAN_Activation error\n");
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) {
        printf("FIFO1 CAN_Activation error\n");
        Error_Handler();
    }

    printf("CAN interrupt activated\n");

    while ((!dynamixel_kondo_ctrl_global[0]._is_initialized) && (!dynamixel_kondo_ctrl_global[1]._is_initialized) && (!dynamixel_kondo_ctrl_global[2]._is_initialized) && (!dynamixel_kondo_ctrl_global[3]._is_initialized) && (!dynamixel_kondo_ctrl_global[4]._is_initialized) && (!dynamixel_kondo_ctrl_global[5]._is_initialized) && (!dynamixel_kondo_ctrl_global[6]._is_initialized) && (!dynamixel_kondo_ctrl_global[7]._is_initialized)) { // mainから初期化命令が送られて来るまで待機
        if (HAL_CAN_GetError(hcan) == HAL_CAN_ERROR_BOF) {
            HAL_CAN_Init(hcan);
        }
        SendBytes(hcan, Make_CAN_ID(NODE_DYNAMIXEL_KONDO, node_id, 0, DYNAMIXEL_KONDO_CMD_AWAKE), &node_id, sizeof(uint8_t));
        printf("send awake cmd\n");
        HAL_Delay(1000);
    }
    for (int i = 0; i < 8; i++) {
        printf("-------dynamixel_kondo%d-------\n", i);
        CANDynamixel_Kondo_Print(&dynamixel_kondo_ctrl_global[i]);
    }
}

typedef struct {
    uint8_t device_id;
    uint8_t cmd;
    uint8_t rxData[8];
} CANRxBuf;

CANRxBuf buffer[CAN_RXBUFFER_SIZE];
uint32_t readpoint = 0;
uint32_t writepoint = 0;
uint8_t isfull = 0;

HAL_StatusTypeDef PushTx8Bytes(uint8_t device_id, uint8_t cmd, uint8_t *rxData) {
    buffer[writepoint].device_id = device_id;
    buffer[writepoint].cmd = cmd;
    for (uint8_t i = 0; i < 8; i++) buffer[writepoint].rxData[i] = rxData[i];

    if (isfull == 1) readpoint = (readpoint + 1) & (CAN_RXBUFFER_SIZE - 1);
    writepoint = (writepoint + 1) & (CAN_RXBUFFER_SIZE - 1);

    if (writepoint == readpoint) {
        isfull = 1;
    }

    return HAL_OK;
}

HAL_StatusTypeDef Process_Fifo0Msg(void) {
    while (isfull != 0 || readpoint != writepoint) {
        float fdata[2];
        uint8_t device_id = buffer[readpoint].device_id;
        switch (buffer[readpoint].cmd) {
            case DYNAMIXEL_KONDO_CMD_INIT1:
                if (dynamixel_kondo_ctrl_global[device_id]._is_initialized) return HAL_ERROR;
                dynamixel_kondo_ctrl_global[device_id].dynamixel_or_kondo = buffer[readpoint].rxData[0];
                if (dkboard_type == DKBOARD_UNINITIALIZED)
                    dkboard_type = dynamixel_kondo_ctrl_global[device_id].dynamixel_or_kondo;

                switch (dynamixel_kondo_ctrl_global[device_id].dynamixel_or_kondo) {
                    case DKBOARD_INITIALIZED_AS_DYNAMIXEL:
                        dynamixel_kondo_ctrl_global[device_id].dynamixel_param.model = buffer[readpoint].rxData[1];
                        dynamixel_kondo_ctrl_global[device_id].dynamixel_param.id = buffer[readpoint].rxData[2];
                        CANDynamixel_SetRotDir(&dynamixel_kondo_ctrl_global[device_id], buffer[readpoint].rxData[3]);
                        break;
                    case DKBOARD_INITIALIZED_AS_KONDO:
                        dynamixel_kondo_ctrl_global[device_id].kondo_param.id = buffer[readpoint].rxData[1];
                        break;
                    default:
                        break;
                }
                break;
            case DYNAMIXEL_KONDO_CMD_INIT2:
                if (dynamixel_kondo_ctrl_global[device_id]._is_initialized) return HAL_ERROR;
                memcpy(&fdata, buffer[readpoint].rxData, sizeof(fdata));
                switch (dynamixel_kondo_ctrl_global[device_id].dynamixel_or_kondo) {
                    case DKBOARD_INITIALIZED_AS_DYNAMIXEL:
                        dynamixel_kondo_ctrl_global[device_id].dynamixel_param.quant_per_degree = fdata[0];
                        break;
                    case DKBOARD_INITIALIZED_AS_KONDO:
                        dynamixel_kondo_ctrl_global[device_id].kondo_param.quant_per_degree = fdata[0];
                        break;
                    default:
                        break;
                }
                initialize_cmd_received = 1;
                if (dynamixel_kondo_ctrl_global[device_id].dynamixel_or_kondo == dkboard_type)
                    dynamixel_kondo_ctrl_global[device_id].need_dynamixel_kondo_initialize = true;
                break;
            case DYNAMIXEL_KONDO_CMD_CHANGE_CTRL1:
                if (!initialize_cmd_received) return HAL_ERROR;
                if (!CANDynamixel_Kondo_Change_Status(&dynamixel_kondo_ctrl_global[device_id], DYNAMIXEL_KONDO_STATUS_CHANGE_CTRL)) return HAL_ERROR;
                memcpy(&fdata, buffer[readpoint].rxData, sizeof(fdata));
                dynamixel_kondo_ctrl_global[device_id]._start_change_ctrl = 1;
                dynamixel_kondo_ctrl_global[device_id]._finish_change_ctrl = 0;
                switch (dynamixel_kondo_ctrl_global[device_id].dynamixel_or_kondo) {
                    case DKBOARD_INITIALIZED_AS_DYNAMIXEL:
                        dynamixel_kondo_ctrl_global[device_id].dynamixel_param.ctrl_param.PID_VEL_param.ki = fdata[0];
                        dynamixel_kondo_ctrl_global[device_id].dynamixel_param.ctrl_param.PID_VEL_param.kp = fdata[1];
                        break;
                    case DKBOARD_INITIALIZED_AS_KONDO:
                        break;
                    default:
                        break;
                }
                break;
            case DYNAMIXEL_KONDO_CMD_CHANGE_CTRL2:
                if (!initialize_cmd_received) return HAL_ERROR;
                if (!dynamixel_kondo_ctrl_global[device_id]._start_change_ctrl) return HAL_ERROR;
                memcpy(&fdata, buffer[readpoint].rxData, sizeof(fdata));
                switch (dynamixel_kondo_ctrl_global[device_id].dynamixel_or_kondo) {
                    case DKBOARD_INITIALIZED_AS_DYNAMIXEL:
                        dynamixel_kondo_ctrl_global[device_id].dynamixel_param.ctrl_param.PID_POS_param.kd = fdata[0];
                        dynamixel_kondo_ctrl_global[device_id].dynamixel_param.ctrl_param.PID_POS_param.ki = fdata[1];
                        break;
                    case DKBOARD_INITIALIZED_AS_KONDO:
                        dynamixel_kondo_ctrl_global[device_id].kondo_param.ctrl_param.PID_EXPOS_param.kd = fdata[0];
                        dynamixel_kondo_ctrl_global[device_id].kondo_param.ctrl_param.PID_EXPOS_param.ki = fdata[1];
                        break;
                    default:
                        break;
                }
                break;
            case DYNAMIXEL_KONDO_CMD_CHANGE_CTRL3:
                if (!initialize_cmd_received) return HAL_ERROR;
                if (!dynamixel_kondo_ctrl_global[device_id]._start_change_ctrl) return HAL_ERROR;
                memcpy(&fdata, buffer[readpoint].rxData, sizeof(fdata));
                switch (dynamixel_kondo_ctrl_global[device_id].dynamixel_or_kondo) {
                    case DKBOARD_INITIALIZED_AS_DYNAMIXEL:
                        dynamixel_kondo_ctrl_global[device_id].dynamixel_param.ctrl_param.PID_POS_param.kp = fdata[0];
                        break;
                    case DKBOARD_INITIALIZED_AS_KONDO:
                        dynamixel_kondo_ctrl_global[device_id].kondo_param.ctrl_param.PID_EXPOS_param.kp = fdata[0];
                        break;
                    default:
                        break;
                }
                break;
            case DYNAMIXEL_KONDO_CMD_CHANGE_CTRL4:
                if (!initialize_cmd_received) return HAL_ERROR;
                if (!dynamixel_kondo_ctrl_global[device_id]._start_change_ctrl) return HAL_ERROR;
                switch (dynamixel_kondo_ctrl_global[device_id].dynamixel_or_kondo) {
                    case DKBOARD_INITIALIZED_AS_DYNAMIXEL:
                        dynamixel_kondo_ctrl_global[device_id].dynamixel_param.ctrl_param.ctrl_type = buffer[readpoint].rxData[0];
                        dynamixel_kondo_ctrl_global[device_id].dynamixel_param.ctrl_param.feedback = buffer[readpoint].rxData[1];
                        dynamixel_kondo_ctrl_global[device_id].dynamixel_param.fb_type = buffer[readpoint].rxData[2];
                        break;
                    case DKBOARD_INITIALIZED_AS_KONDO:
                        dynamixel_kondo_ctrl_global[device_id].kondo_param.ctrl_param.ctrl_type = buffer[readpoint].rxData[0];
                        dynamixel_kondo_ctrl_global[device_id].kondo_param.ctrl_param.feedback = buffer[readpoint].rxData[1];
                        dynamixel_kondo_ctrl_global[device_id].kondo_param.fb_type = buffer[readpoint].rxData[2];
                        break;
                    default:
                        break;
                }
                break;
            case DYNAMIXEL_KONDO_CMD_CHANGE_CTRL5:
                if (!initialize_cmd_received) return HAL_ERROR;
                if (!dynamixel_kondo_ctrl_global[device_id]._start_change_ctrl) return HAL_ERROR;
                memcpy(&fdata, buffer[readpoint].rxData, sizeof(fdata));
                switch (dynamixel_kondo_ctrl_global[device_id].dynamixel_or_kondo) {
                    case DKBOARD_INITIALIZED_AS_DYNAMIXEL:
                        dynamixel_kondo_ctrl_global[device_id].dynamixel_param.ctrl_param.profile_acceleration = fdata[0];
                        dynamixel_kondo_ctrl_global[device_id].dynamixel_param.ctrl_param.profile_velocity = fdata[1];
                        break;
                    case DKBOARD_INITIALIZED_AS_KONDO:
                        dynamixel_kondo_ctrl_global[device_id].kondo_param.ctrl_param.trajectory_type = buffer[readpoint].rxData[0];
                        break;
                    default:
                        break;
                }
                dynamixel_kondo_ctrl_global[device_id]._start_change_ctrl = 0;
                dynamixel_kondo_ctrl_global[device_id]._finish_change_ctrl = 1;
                dynamixel_kondo_ctrl_global[device_id]._is_initialized = 1; // end init
                break;
            case DYNAMIXEL_KONDO_CMD_ENABLE:
                if (!CANDynamixel_Kondo_Change_Status(&dynamixel_kondo_ctrl_global[device_id], DYNAMIXEL_KONDO_STATUS_ENABLE)) return HAL_ERROR;
                break;
            case DYNAMIXEL_KONDO_CMD_DISABLE:
                if (!CANDynamixel_Kondo_Change_Status(&dynamixel_kondo_ctrl_global[device_id], DYNAMIXEL_KONDO_STATUS_DISABLE)) return HAL_ERROR;
                break;
            case DYNAMIXEL_KONDO_CMD_SET_TARGET:
                memcpy(&fdata, buffer[readpoint].rxData, sizeof(fdata));
                dynamixel_kondo_ctrl_global[device_id].target_value = fdata[0];
                dynamixel_kondo_ctrl_global[device_id].target_time = fdata[1];
                break;
            default:
                break;
        }
        // Pop
        readpoint = (readpoint + 1) & (CAN_RXBUFFER_SIZE - 1);
        isfull = 0;
    }

    return HAL_OK;
    ;
}

void WhenCANRxFifo0MsgPending() { // Fifo0MsgPendingで呼び出すこと. CAN受信時に呼び出される
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    // Get RX message
    if (HAL_CAN_GetRxMessage(phcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
        // Reception Error
        printf("GetRxMessage error\n");
        Error_Handler();
    }
    CAN_Device can_device = Extract_CAN_Device(rxHeader.ExtId);
    uint8_t extracted_cmd = Extract_CAN_CMD(rxHeader.ExtId);
    if ((can_device.node_type != NODE_DYNAMIXEL_KONDO) || (can_device.node_id != node_id)) {
        printf("warn get msg to another device\n");
        return; // 別のdeviceへの命令を受信してる
    }
    uint8_t device_id = can_device.device_num;
    PushTx8Bytes(device_id, extracted_cmd, rxData);
}

void WhenCANRxFifo1MsgPending() { // Fifo1MsgPendingで呼び出すこと. CAN受信時に呼び出される
    // 現在は使っていない
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    // Get RX message
    if (HAL_CAN_GetRxMessage(phcan, CAN_RX_FIFO1, &rxHeader, rxData) != HAL_OK) {
        // Reception Error
        printf("GetRxMessage error\n");
        Error_Handler();
    }
}

uint8_t _GetID(void) {                                                                                                                                                      // 基板のnode idを取得する
    uint8_t _node_id = ((!HAL_GPIO_ReadPin(ID1_GPIO_Port, ID1_Pin)) | (!HAL_GPIO_ReadPin(ID2_GPIO_Port, ID2_Pin) << 1) | (!HAL_GPIO_ReadPin(ID4_GPIO_Port, ID4_Pin) << 2)); // calibの仕様上node_idは3bit
    printf("id:%d\n", _node_id);
    return _node_id;
}

void CANDynamixel_Kondo_Update(Dynamixel_Kondo_Ctrl_Typedef *can_dynamixel_kondo) { // この関数でDynamixel_Kondoに目標値を送ってる
    if (_is_usb_connected()) {
        printf("USB is connected!\n");
        return;
    }

    if (can_dynamixel_kondo->need_dynamixel_kondo_initialize) {
        switch (can_dynamixel_kondo->dynamixel_or_kondo) {
            case DKBOARD_INITIALIZED_AS_DYNAMIXEL:
                if (!is_rs485_system_initialized) {
                    Init_Dynamixel_RS485_System(phuart);
                    is_rs485_system_initialized = true;
                }
                Dynamixel_init(&(can_dynamixel_kondo->dynamixel), can_dynamixel_kondo->dynamixel_param.id, _CANDynamixel_getModelNumber(can_dynamixel_kondo->dynamixel_param.model));
                break;
            case DKBOARD_INITIALIZED_AS_KONDO:
                if (!is_rs485_system_initialized) {
                    Init_Kondo_RS485_System(phuart);
                    is_rs485_system_initialized = true;
                }
                Kondo_init(&(can_dynamixel_kondo->kondo), can_dynamixel_kondo->kondo_param.id);
                break;
            default:
                break;
        }
        can_dynamixel_kondo->need_dynamixel_kondo_initialize = false;
    }

    if (can_dynamixel_kondo->need_dynamixel_kondo_enable) {
        _CANDynamixel_Kondo_Enable(can_dynamixel_kondo);
        if (can_dynamixel_kondo->dynamixel_or_kondo == DKBOARD_INITIALIZED_AS_KONDO)
            can_dynamixel_kondo->_last_target_pos = 1.0e32f;
        can_dynamixel_kondo->need_dynamixel_kondo_enable = false;
    }

    if (can_dynamixel_kondo->need_dynamixel_kondo_disable) {
        _CANDynamixel_Kondo_Disable(can_dynamixel_kondo);
        can_dynamixel_kondo->need_dynamixel_kondo_disable = false;
    }

    if ((can_dynamixel_kondo->status) == DYNAMIXEL_KONDO_STATUS_ENABLE) { // Dynamixel や Kondo が Enable の時の動作

        if (isnan(can_dynamixel_kondo->target_value)) return;

        switch (can_dynamixel_kondo->dynamixel_or_kondo) {
            case DKBOARD_INITIALIZED_AS_DYNAMIXEL:
                Dynamixel_torqueEnable(&can_dynamixel_kondo->dynamixel, DYNAMIXEL_TORQUE_ENABLE);

                if (can_dynamixel_kondo->dynamixel.series == DYNAMIXEL_MX) {
                    Dynamixel_setProfileAcceleration(&can_dynamixel_kondo->dynamixel, can_dynamixel_kondo->dynamixel_param.ctrl_param.profile_acceleration);
                    Dynamixel_setProfileVelocity(&can_dynamixel_kondo->dynamixel, can_dynamixel_kondo->dynamixel_param.ctrl_param.profile_velocity);
                }

                switch (can_dynamixel_kondo->dynamixel_param.ctrl_param.ctrl_type) {
                    case DYNAMIXEL_CTRL_POS: // 位置制御
                        Dynamixel_goalPosition(&can_dynamixel_kondo->dynamixel, can_dynamixel_kondo->target_value / can_dynamixel_kondo->dynamixel_param.quant_per_degree);
                        break;
                    case DYNAMIXEL_CTRL_VEL: // 速度制御
                        Dynamixel_goalVelocity(&can_dynamixel_kondo->dynamixel, can_dynamixel_kondo->target_value / can_dynamixel_kondo->dynamixel_param.quant_per_degree);
                        break;
                    case DYNAMIXEL_CTRL_CUR: // 電流制御
                        Dynamixel_goalCurrent(&can_dynamixel_kondo->dynamixel, can_dynamixel_kondo->target_value);
                        break;
                    case DYNAMIXEL_CTRL_EXPOS: // 拡張位置制御
                        Dynamixel_goalPosition(&can_dynamixel_kondo->dynamixel, can_dynamixel_kondo->target_value);
                        break;
                    case DYNAMIXEL_CTRL_CUPOS: // 電流に基づく速度制御
                        Dynamixel_goalPosition(&can_dynamixel_kondo->dynamixel, can_dynamixel_kondo->target_value);
                        break;
                    case DYNAMIXEL_CTRL_PWM: // PWM制御
                        Dynamixel_goalPWM(&can_dynamixel_kondo->dynamixel, can_dynamixel_kondo->target_value);
                        break;
                    default:
                        break;
                }

                break;
            case DKBOARD_INITIALIZED_AS_KONDO:
                Kondo_runMode(&can_dynamixel_kondo->kondo, KONDO_RUN_MODE_NORMAL);
                Kondo_trajectoryType(&can_dynamixel_kondo->kondo, can_dynamixel_kondo->kondo_param.ctrl_param.trajectory_type);

                switch (can_dynamixel_kondo->kondo_param.ctrl_param.ctrl_type) {
                    case KONDO_CTRL_POS: // 位置制御
                        if (can_dynamixel_kondo->target_value != can_dynamixel_kondo->_last_target_pos)
                            can_dynamixel_kondo->need_kondo_set_position = true;
                        if (can_dynamixel_kondo->need_kondo_set_position) {
                            const Kondo_ErrorCode err = Kondo_goalPosition_withTime(&can_dynamixel_kondo->kondo, can_dynamixel_kondo->target_value / can_dynamixel_kondo->kondo_param.quant_per_degree, can_dynamixel_kondo->target_time);
                            if (err == KONDO_SUCCESS) {
                                can_dynamixel_kondo->need_kondo_set_position = false;
                                can_dynamixel_kondo->_last_target_pos = can_dynamixel_kondo->target_value;
                            }
                        }

                        break;
                    case KONDO_CTRL_VEL: // 速度制御
                        Kondo_goalVelocity(&can_dynamixel_kondo->kondo, can_dynamixel_kondo->target_value / can_dynamixel_kondo->kondo_param.quant_per_degree);
                        break;
                    case KONDO_CTRL_TOR: // トルク制御
                        Kondo_goalTorque(&can_dynamixel_kondo->kondo, can_dynamixel_kondo->target_value);
                        break;
                    case KONDO_CTRL_EXPOS: { // 拡張位置制御
                        can_dynamixel_kondo->kondo_expos_ctrl_target_vel = 0.0f;
                        const float dt = 10.0e-3f; // 制御周期 [s] // TODO:
                        if (!isnan(can_dynamixel_kondo->kondo_extended_position_calibrated)) {
                            const float pos_error = (can_dynamixel_kondo->target_value - can_dynamixel_kondo->kondo_extended_position_calibrated) / 360.0f; // [rot]

                            can_dynamixel_kondo->kondo_param.ctrl_param.PID_EXPOS_param.integral =
                                clip_abs_float(
                                    can_dynamixel_kondo->kondo_param.ctrl_param.PID_EXPOS_param.integral + pos_error * dt,
                                    can_dynamixel_kondo->kondo_param.ctrl_param.integral_limit);

                            can_dynamixel_kondo->kondo_expos_ctrl_target_vel =
                                clip_abs_float(
                                    pos_error * can_dynamixel_kondo->kondo_param.ctrl_param.PID_EXPOS_param.kp + can_dynamixel_kondo->kondo_param.ctrl_param.PID_EXPOS_param.integral * can_dynamixel_kondo->kondo_param.ctrl_param.PID_EXPOS_param.ki,
                                    can_dynamixel_kondo->kondo_param.ctrl_param.velocity_limit);
                        }
                        Kondo_goalVelocity(&can_dynamixel_kondo->kondo, can_dynamixel_kondo->kondo_expos_ctrl_target_vel);
                        break;
                    }
                    default:
                        break;
                }

                break;
            default:
                break;
        }

    } else if ((can_dynamixel_kondo->status) == DYNAMIXEL_KONDO_STATUS_DISABLE || (can_dynamixel_kondo->status) == DYNAMIXEL_KONDO_STATUS_CHANGE_CTRL) {
        switch (can_dynamixel_kondo->dynamixel_or_kondo) {
            case DKBOARD_INITIALIZED_AS_DYNAMIXEL:
                Dynamixel_torqueEnable(&can_dynamixel_kondo->dynamixel, DYNAMIXEL_TORQUE_DISABLE);
                break;
            case DKBOARD_INITIALIZED_AS_KONDO:
                Kondo_runMode(&can_dynamixel_kondo->kondo, KONDO_RUN_MODE_FREE);
                break;
            default:
                break;
        }
    }
}

static void CANDynamixel_SetRotDir(Dynamixel_Kondo_Ctrl_Typedef *can_dynamixel_kondo, Dynamixel_DIR dir) {
    can_dynamixel_kondo->dynamixel_param.rot_dir = dir;
    // TODO : rotdir
    if (can_dynamixel_kondo->dynamixel_param.rot_dir == DYNAMIXEL_DIR_FW) {
        ; // FW
    } else {
        ; // BC
    }
}

uint8_t CANDynamixel_Kondo_Send_Feedback_Main(Dynamixel_Kondo_Ctrl_Typedef *can_dynamixel_kondo) { // MainのマイコンにFeedbackを送信する関数（戻り値：送ったら0, 送らなかったら1）
    switch (can_dynamixel_kondo->dynamixel_or_kondo) {
        case DKBOARD_INITIALIZED_AS_DYNAMIXEL:
            if (can_dynamixel_kondo->dynamixel_param.ctrl_param.feedback != DYNAMIXEL_FB_ENABLE) return 1;
            break;
        case DKBOARD_INITIALIZED_AS_KONDO:
            if (can_dynamixel_kondo->kondo_param.ctrl_param.feedback != KONDO_FB_ENABLE) return 1;
            break;
        default:
            break;
    }
    if (can_dynamixel_kondo->status == DYNAMIXEL_KONDO_STATUS_INIT) return 1;

    if (can_dynamixel_kondo->need_dynamixel_kondo_initialize) return 1; // まだ DYNAMIXEL や KONDO が初期化されていないときには FB を送らない

    if (_is_usb_connected()) {
        printf("USB is connected!\n");
        return 1;
    }

    CANDynamixel_Kondo_Feedback_Typedef tmp = {
        .status = can_dynamixel_kondo->status,
        .fb_type = can_dynamixel_kondo->dynamixel_param.fb_type,
        .value = NAN,
    };

    switch (can_dynamixel_kondo->dynamixel_or_kondo) {
        case DKBOARD_INITIALIZED_AS_DYNAMIXEL: {
            switch (can_dynamixel_kondo->dynamixel_param.fb_type) {
                case DYNAMIXEL_FB_POS:
                    Dynamixel_presentPosition(&can_dynamixel_kondo->dynamixel, &can_dynamixel_kondo->dynamixel_fb);
                    tmp.value = can_dynamixel_kondo->dynamixel_fb.position;
                    break;
                case DYNAMIXEL_FB_VEL:
                    Dynamixel_presentVelocity(&can_dynamixel_kondo->dynamixel, &can_dynamixel_kondo->dynamixel_fb);
                    tmp.value = can_dynamixel_kondo->dynamixel_fb.velocity;
                    break;
                case DYNAMIXEL_FB_CUR:
                    Dynamixel_presentCurrent(&can_dynamixel_kondo->dynamixel, &can_dynamixel_kondo->dynamixel_fb);
                    tmp.value = can_dynamixel_kondo->dynamixel_fb.current;
                    break;
                case DYNAMIXEL_FB_MOV:
                    Dynamixel_moving(&can_dynamixel_kondo->dynamixel, &can_dynamixel_kondo->dynamixel_fb);
                    tmp.value = can_dynamixel_kondo->dynamixel_fb.moving;
                    break;
                case DYNAMIXEL_FB_VOL:
                    Dynamixel_presentVoltage(&can_dynamixel_kondo->dynamixel, &can_dynamixel_kondo->dynamixel_fb);
                    tmp.value = can_dynamixel_kondo->dynamixel_fb.voltage;
                    break;
                case DYNAMIXEL_FB_PWM:
                    Dynamixel_presentPWM(&can_dynamixel_kondo->dynamixel, &can_dynamixel_kondo->dynamixel_fb);
                    tmp.value = can_dynamixel_kondo->dynamixel_fb.pwm;
                    break;
                case DYNAMIXEL_FB_TMP:
                    Dynamixel_presentTemperature(&can_dynamixel_kondo->dynamixel, &can_dynamixel_kondo->dynamixel_fb);
                    tmp.value = can_dynamixel_kondo->dynamixel_fb.temperature;
                    break;
                case DYNAMIXEL_FB_LOA:
                    Dynamixel_presentLoad(&can_dynamixel_kondo->dynamixel, &can_dynamixel_kondo->dynamixel_fb);
                    tmp.value = can_dynamixel_kondo->dynamixel_fb.load;
                    break;
                default:
                    tmp.value = NAN;
                    break;
            }

            break;
        }
        case DKBOARD_INITIALIZED_AS_KONDO: {
            switch (can_dynamixel_kondo->kondo_param.fb_type) {
                case KONDO_FB_POS:
                    Kondo_presentPosition(&can_dynamixel_kondo->kondo, &can_dynamixel_kondo->kondo_fb);
                    tmp.value = can_dynamixel_kondo->kondo_fb.position;
                    break;
                case KONDO_FB_VEL:
                    Kondo_presentVelocity(&can_dynamixel_kondo->kondo, &can_dynamixel_kondo->kondo_fb);
                    tmp.value = can_dynamixel_kondo->kondo_fb.velocity;
                    break;
                case KONDO_FB_CUR:
                    Kondo_presentCurrent(&can_dynamixel_kondo->kondo, &can_dynamixel_kondo->kondo_fb);
                    tmp.value = can_dynamixel_kondo->kondo_fb.current;
                    break;
                case KONDO_FB_VOL:
                    Kondo_presentVoltage(&can_dynamixel_kondo->kondo, &can_dynamixel_kondo->kondo_fb);
                    tmp.value = can_dynamixel_kondo->kondo_fb.voltage;
                    break;
                case KONDO_FB_TMP:
                    Kondo_presentTemperature(&can_dynamixel_kondo->kondo, &can_dynamixel_kondo->kondo_fb);
                    tmp.value = can_dynamixel_kondo->kondo_fb.temperature;
                    break;
                case KONDO_FB_EXPOS:
                    Kondo_presentExtendedPosition(&can_dynamixel_kondo->kondo, &can_dynamixel_kondo->kondo_fb);

                    if (isnan(can_dynamixel_kondo->kondo_extended_position_offset)) {
                        if (Kondo_presentPosition(&can_dynamixel_kondo->kondo, &can_dynamixel_kondo->kondo_fb) == KONDO_SUCCESS) {
                            can_dynamixel_kondo->kondo_extended_position_offset =
                                can_dynamixel_kondo->kondo_fb.extended_position - can_dynamixel_kondo->kondo_fb.position;
                        }
                    }

                    can_dynamixel_kondo->kondo_extended_position_calibrated =
                        can_dynamixel_kondo->kondo_fb.extended_position - can_dynamixel_kondo->kondo_extended_position_offset;
                    tmp.value = can_dynamixel_kondo->kondo_extended_position_calibrated;
                    break;
                default:
                    tmp.value = NAN;
                    break;
            }

            break;
        }
        default:
            break;
    }

    if (SendBytes(phcan, Make_CAN_ID_from_CAN_Device(&(can_dynamixel_kondo->dynamixel_param.device), DYNAMIXEL_KONDO_CMD_FB), (uint8_t *)&tmp, sizeof(CANDynamixel_Kondo_Feedback_Typedef)) != HAL_OK) { // 送信
        Error_Handler();
    }

    return 0;
}

static bool CANDynamixel_Kondo_Change_Status(Dynamixel_Kondo_Ctrl_Typedef *can_dynamixel_kondo, Dynamixel_Kondo_Status next_status) {
    Dynamixel_Kondo_Status present_status = can_dynamixel_kondo->status; // 現在の状態

    // printf("[CANDynamixel_Kondo_Change_Status()] prev=%s, next=%s\n",
    //        _string_dynamixel_kondo_status(present_status),
    //        _string_dynamixel_kondo_status(next_status));

    can_dynamixel_kondo->incorrect_status_cmd_receive_counter_global += 1;
    switch (next_status) { // ここで状態遷移できるかを判定し, 可能なら遷移している
        case DYNAMIXEL_KONDO_STATUS_ENABLE:
            if ((present_status != DYNAMIXEL_KONDO_STATUS_CHANGE_CTRL) && (present_status != DYNAMIXEL_KONDO_STATUS_DISABLE)) return false;
            can_dynamixel_kondo->need_dynamixel_kondo_enable = true;
            break;
        case DYNAMIXEL_KONDO_STATUS_DISABLE:
            if (present_status != DYNAMIXEL_KONDO_STATUS_ENABLE) return false;
            can_dynamixel_kondo->need_dynamixel_kondo_disable = true;
            break;
        case DYNAMIXEL_KONDO_STATUS_CHANGE_CTRL:
            if ((present_status != DYNAMIXEL_KONDO_STATUS_INIT) && (present_status != DYNAMIXEL_KONDO_STATUS_CHANGE_CTRL) && (present_status != DYNAMIXEL_KONDO_STATUS_DISABLE)) return false;
            can_dynamixel_kondo->status = DYNAMIXEL_KONDO_STATUS_CHANGE_CTRL;
            break;
        default:
            break;
    }
    can_dynamixel_kondo->incorrect_status_cmd_receive_counter_global -= 1;
    return true;
}

static void _CANDynamixel_Kondo_Enable(Dynamixel_Kondo_Ctrl_Typedef *can_dynamixel_kondo) {
    if (_is_usb_connected()) {
        printf("USB is connected!\n");
        return;
    };
    // TODO : PIDパラメータ

    switch (can_dynamixel_kondo->dynamixel_or_kondo) {
        case DKBOARD_INITIALIZED_AS_DYNAMIXEL:
            switch (can_dynamixel_kondo->dynamixel_param.ctrl_param.ctrl_type) {
                case DYNAMIXEL_CTRL_POS:
                    Dynamixel_operatingMode(&can_dynamixel_kondo->dynamixel, DYNAMIXEL_POSITION_CTRL_MODE);
                    break;
                case DYNAMIXEL_CTRL_VEL:
                    Dynamixel_operatingMode(&can_dynamixel_kondo->dynamixel, DYNAMIXEL_VELOCITY_CTRL_MODE);
                    break;
                case DYNAMIXEL_CTRL_CUR:
                    Dynamixel_operatingMode(&can_dynamixel_kondo->dynamixel, DYNAMIXEL_CURRENT_CTRL_MODE);
                    break;
                case DYNAMIXEL_CTRL_EXPOS:
                    Dynamixel_operatingMode(&can_dynamixel_kondo->dynamixel, DYNAMIXEL_EXTENDED_POSITION_CTRL_MODE);
                    break;
                case DYNAMIXEL_CTRL_CUPOS:
                    Dynamixel_operatingMode(&can_dynamixel_kondo->dynamixel, DYNAMIXEL_CURRENT_BASED_POSITION_CTRL_MODE);
                    break;
                case DYNAMIXEL_CTRL_PWM:
                    Dynamixel_operatingMode(&can_dynamixel_kondo->dynamixel, DYNAMIXEL_PWM_CTRL_MODE);
                    break;
            }
            Dynamixel_torqueEnable(&can_dynamixel_kondo->dynamixel, DYNAMIXEL_TORQUE_ENABLE);

            break;
        case DKBOARD_INITIALIZED_AS_KONDO:
            switch (can_dynamixel_kondo->kondo_param.ctrl_param.ctrl_type) {
                case KONDO_CTRL_POS:
                    Kondo_controlMode(&can_dynamixel_kondo->kondo, KONDO_CTRL_MODE_POSITION);
                    break;
                case KONDO_CTRL_VEL:
                    Kondo_controlMode(&can_dynamixel_kondo->kondo, KONDO_CTRL_MODE_VELOCITY);
                    break;
                case KONDO_CTRL_TOR:
                    Kondo_controlMode(&can_dynamixel_kondo->kondo, KONDO_CTRL_MODE_TORQUE);
                    break;
                case KONDO_CTRL_EXPOS:
                    Kondo_controlMode(&can_dynamixel_kondo->kondo, KONDO_CTRL_MODE_VELOCITY); // ※拡張位置制御では KONDO の速度制御モードを使用する
                    break;
            }
            Kondo_runMode(&can_dynamixel_kondo->kondo, KONDO_RUN_MODE_NORMAL);

            break;
        default:
            break;
    }

    can_dynamixel_kondo->status = DYNAMIXEL_KONDO_STATUS_ENABLE;
}

static void _CANDynamixel_Kondo_Disable(Dynamixel_Kondo_Ctrl_Typedef *can_dynamixel_kondo) {
    if (_is_usb_connected()) {
        printf("USB is connected!\n");
        return;
    }
    switch (can_dynamixel_kondo->dynamixel_or_kondo) {
        case DKBOARD_INITIALIZED_AS_DYNAMIXEL:
            Dynamixel_torqueEnable(&can_dynamixel_kondo->dynamixel, DYNAMIXEL_TORQUE_DISABLE);
            break;
        case DKBOARD_INITIALIZED_AS_KONDO:
            Kondo_runMode(&can_dynamixel_kondo->kondo, KONDO_RUN_MODE_FREE);
            break;
        default:
            break;
    }
    can_dynamixel_kondo->status = DYNAMIXEL_KONDO_STATUS_DISABLE;
}

void CANDynamixel_Kondo_Print(Dynamixel_Kondo_Ctrl_Typedef *can_dynamixel_kondo) {
    printf("{ ");
    switch (can_dynamixel_kondo->dynamixel_or_kondo) {
        case DKBOARD_UNINITIALIZED:
            printf("UNINITIALIZED");
            break;
        case DKBOARD_INITIALIZED_AS_DYNAMIXEL:
            printf("DYNAMIXEL");
            break;
        case DKBOARD_INITIALIZED_AS_KONDO:
            printf("KONDO");
            break;
    }
    printf(": ");
    printf("status=%s, ", _string_dynamixel_kondo_status(can_dynamixel_kondo->status));
    switch (can_dynamixel_kondo->dynamixel_or_kondo) {
        case DKBOARD_INITIALIZED_AS_DYNAMIXEL:
            printf("id=%d, ", can_dynamixel_kondo->dynamixel_param.id);
            printf("model=0x%04x, ", can_dynamixel_kondo->dynamixel.model_specified);
            printf("target=%.2f, ", can_dynamixel_kondo->target_value);
            switch (can_dynamixel_kondo->dynamixel_param.ctrl_param.ctrl_type) {
                case DYNAMIXEL_CTRL_POS:
                case DYNAMIXEL_CTRL_EXPOS:
                case DYNAMIXEL_CTRL_CUPOS:
                    printf("pos=%.2f", can_dynamixel_kondo->dynamixel_fb.position);
                    break;
                case DYNAMIXEL_CTRL_VEL:
                    printf("vel=%.2f", can_dynamixel_kondo->dynamixel_fb.velocity);
                    break;
                case DYNAMIXEL_CTRL_CUR:
                    printf("cur=%.2f", can_dynamixel_kondo->dynamixel_fb.current);
                    break;
                case DYNAMIXEL_CTRL_PWM:
                    printf("pwm=%.2f", can_dynamixel_kondo->dynamixel_fb.pwm);
                    break;
            }
            printf(", ");
            printf("pos_kp=%.2f, ", can_dynamixel_kondo->dynamixel_param.ctrl_param.PID_POS_param.kp);
            printf("fb_type=%s, ", _string_dynamixel_kondo_fb_type(can_dynamixel_kondo->dynamixel_param.fb_type));
            break;
        case DKBOARD_INITIALIZED_AS_KONDO:
            printf("id=%d, ", can_dynamixel_kondo->kondo_param.id);
            printf("target=%.2f, ", can_dynamixel_kondo->target_value);
            switch (can_dynamixel_kondo->kondo_param.ctrl_param.ctrl_type) {
                case KONDO_CTRL_POS:
                    printf("pos=%.2f, ", can_dynamixel_kondo->kondo_fb.position);
                    printf("traj_type=%s, ", _string_kondo_traj(can_dynamixel_kondo->kondo_param.ctrl_param.trajectory_type));
                    printf("goal_time=%.2f s, ", can_dynamixel_kondo->target_time);
                    break;
                case KONDO_CTRL_VEL:
                    printf("vel=%.2f, ", can_dynamixel_kondo->kondo_fb.velocity);
                    break;
                case KONDO_CTRL_TOR:
                    printf("cur=%.2f, ", can_dynamixel_kondo->kondo_fb.current);
                    break;
                case KONDO_CTRL_EXPOS:
                    printf("expos=%.2f, ", can_dynamixel_kondo->kondo_extended_position_calibrated);
                    printf("cmdvel=%.2f, ", can_dynamixel_kondo->kondo_expos_ctrl_target_vel);
                    break;
            }
            printf("fb_type=%s, ", _string_dynamixel_kondo_fb_type(can_dynamixel_kondo->kondo_param.fb_type));
            break;
        default:
            break;
    }
    printf("er_cmd=%d ", (int)can_dynamixel_kondo->incorrect_status_cmd_receive_counter_global);
    printf("}\n");
}

static bool _is_usb_connected(void) {
    return (HAL_GPIO_ReadPin(USBEN_GPIO_Port, USBEN_Pin) == GPIO_PIN_SET);
}

static Dynamixel_Model_Number _CANDynamixel_getModelNumber(Dynamixel_MODEL model) {
    switch (model) {
        case DYNAMIXEL_MODEL_EX_106_PLUS:
            return DYNAMIXEL_EX_106_PLUS;
        case DYNAMIXEL_MODEL_MX_28:
            return DYNAMIXEL_MX_28;
        case DYNAMIXEL_MODEL_MX_64:
            return DYNAMIXEL_MX_64;
        case DYNAMIXEL_MODEL_MX_106:
            return DYNAMIXEL_MX_106;
        case DYNAMIXEL_MODEL_DX_117:
            return DYNAMIXEL_DX_117;
        case DYNAMIXEL_MODEL_RX_24F:
            return DYNAMIXEL_RX_24F;
        case DYNAMIXEL_MODEL_RX_64:
            return DYNAMIXEL_RX_64;
        case DYNAMIXEL_MODEL_XM430_W350_R:
            return DYNAMIXEL_XM430_W350_R;
        case DYNAMIXEL_MODEL_XH430_V350_R:
            return DYNAMIXEL_XH430_V350_R;
        case DYNAMIXEL_MODEL_H42_20_S300_R:
            return DYNAMIXEL_H42_20_S300_R;
        default:
            printf("_CANDynamixel_getModelNumber() error!\n");
            return 0;
    }
}

void CANDynamixel_Kondo_UART_Callback(UART_HandleTypeDef *huart) {
    switch (dkboard_type) {
        case DKBOARD_INITIALIZED_AS_DYNAMIXEL:
            Dynamixel_UART_Callback(huart);
            break;
        case DKBOARD_INITIALIZED_AS_KONDO:
            Kondo_UART_Callback(huart);
            break;
        default:
            break;
    }
}

static char *_string_dynamixel_kondo_status(const Dynamixel_Kondo_Status status) {
    switch (status) {
        case DYNAMIXEL_KONDO_STATUS_INIT:
            return "INIT";
        case DYNAMIXEL_KONDO_STATUS_ENABLE:
            return "ENABLE";
        case DYNAMIXEL_KONDO_STATUS_DISABLE:
            return "DISABLE";
        case DYNAMIXEL_KONDO_STATUS_CHANGE_CTRL:
            return "CHANGE CTRL";
        default:
            return "";
    }
}
static char *_string_dynamixel_kondo_fb_type(Dynamixel_Kondo_FB_TYPE fb_type) {
    switch (fb_type) {
        case DYNAMIXEL_FB_POS:
            // case KONDO_FB_POS: // 値は同じ
            return "POS";
        case DYNAMIXEL_FB_VEL:
            // case KONDO_FB_VEL: // 値は同じ
            return "VEL";
        case DYNAMIXEL_FB_CUR:
            // case KONDO_FB_CUR: // 値は同じ
            return "CUR";
        case DYNAMIXEL_FB_MOV:
            return "MOV";
        case DYNAMIXEL_FB_VOL:
            // case KONDO_FB_VOL: // 値は同じ
            return "VOL";
        case DYNAMIXEL_FB_PWM:
            return "PWM";
        case DYNAMIXEL_FB_TMP:
            // case KONDO_FB_TMP: // 値は同じ
            return "TMP";
        case DYNAMIXEL_FB_LOA:
            return "LOA";
        case KONDO_FB_EXPOS:
            return "EXPOS";
        default:
            return "";
    }
}

static char *_string_kondo_traj(Kondo_TRAJECTORY_TYPE traj_type) {
    switch (traj_type) {
        case KONDO_TRAJECTORY_NORMAL:
            return "NORMAL";
        case KONDO_TRAJECTORY_EVEN:
            return "EVEN";
        case KONDO_TRAJECTORY_THIRDPOLY:
            return "3RD";
        case KONDO_TRAJECTORY_FORTHPOLY:
            return "4TH";
        case KONDO_TRAJECTORY_FIFTHPOLY:
            return "5TH";
        default:
            return "";
    }
}

static float clip_abs_float(const float val, const float limit) {
    if (limit > 0.0f) {
        return ((val > limit) ? limit : ((val < -limit) ? -limit : val));
    } else {
        return 0.0f;
    }
}

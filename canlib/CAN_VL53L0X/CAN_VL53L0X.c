/*
 * CAN_VL53L0X.c
 *
 *  Created on: 04 01, 2026
 *      Author: Akitomo KURAKU
 */

// Includes --------------------------------

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "i2c.h"

#include "CAN_VL53L0X.h"

// Variables --------------------------------

static CAN_HandleTypeDef *phcan;
static uint8_t node_id;
static uint8_t initialize_cmd_received = 0;

VL53L0X_Ctrl_Typedef vl53l0x_ctrl_global;

// Private Function Prototypes --------------------------------

static HAL_StatusTypeDef SendBytes(CAN_HandleTypeDef *hcan, uint32_t ExtId, const uint8_t *bytes, uint32_t size);
static HAL_StatusTypeDef PushTx8Bytes(uint8_t device_id, uint8_t cmd, const uint8_t *rxData);
static bool CANVL53L0X_Change_Status(VL53L0X_Ctrl_Typedef *can_vl53l0x, VL53L0X_Status next_status); // 状態遷移可能か判定し, 可能なら遷移する. 状態遷移できた場合はtrueをできなかった場合はfalseを返す. VL53L0X 基板の状態遷移は全てこの関数を用いて行われる

static void _CANVL53L0X_Enable(VL53L0X_Ctrl_Typedef *can_vl53l0x);
static void _CANVL53L0X_Disable(VL53L0X_Ctrl_Typedef *can_vl53l0x);
static uint8_t _GetID(void);

// Functions --------------------------------

static HAL_StatusTypeDef SendBytes(CAN_HandleTypeDef *const hcan, const uint32_t ExtId, const uint8_t *const bytes, const uint32_t size) {
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

void InitCANVL53L0X(CAN_HandleTypeDef *const hcan) {
    node_id = _GetID(); // node_idを取得(基板番号)

    vl53l0x_ctrl_global.vl53l0x_param.device.device_num = 0;
    vl53l0x_ctrl_global.vl53l0x_param.device.node_id = node_id;
    vl53l0x_ctrl_global.vl53l0x_param.device.node_type = NODE_VL53L0X;

    vl53l0x_ctrl_global.distance = NAN;

    // status setting
    vl53l0x_ctrl_global.status = VL53L0X_STATUS_INIT;
    vl53l0x_ctrl_global._is_initialized = 0;
    vl53l0x_ctrl_global._start_change_ctrl = 0;
    vl53l0x_ctrl_global._finish_change_ctrl = 0; // 0

    vl53l0x_ctrl_global.need_vl53l0x_initialize = false;
    vl53l0x_ctrl_global.need_vl53l0x_enable = false;
    vl53l0x_ctrl_global.need_vl53l0x_disable = false;

    // CAN初期化
    CAN_FilterTypeDef sFilterConfig;
    printf("InitCANVL53L0X start\n");

    // ここからCANのフィルター設定
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // IDMASKモード
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 32bitモード
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_VL53L0X, node_id, 0, VL53L0X_CMD_INIT) >> 13;
    sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0b111, 0b111, 0, 0b11111) >> 13;
    sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_VL53L0X, node_id, 0, VL53L0X_CMD_INIT) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0b111, 0b111, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; // Fifo0に受信

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) // フィルターを登録
    {
        /* Filter configuration Error */
        Error_Handler();
    }

    sFilterConfig.FilterBank = 3;
    sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_VL53L0X, node_id, 0, VL53L0X_CMD_CHANGE_CTRL) >> 13;
    sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0b111, 0b111, 0, 0b11111) >> 13;
    sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_VL53L0X, node_id, 0, VL53L0X_CMD_CHANGE_CTRL) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0b111, 0b111, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) {
        /* Filter configuration Error */
        Error_Handler();
    }

    sFilterConfig.FilterBank = 7;
    sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_VL53L0X, node_id, 0, VL53L0X_CMD_ENABLE) >> 13;
    sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0b111, 0b111, 0, 0b11111) >> 13;
    sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_VL53L0X, node_id, 0, VL53L0X_CMD_ENABLE) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0b111, 0b111, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) {
        /* Filter configuration Error */
        Error_Handler();
    }

    sFilterConfig.FilterBank = 8;
    sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_VL53L0X, node_id, 0, VL53L0X_CMD_DISABLE) >> 13;
    sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0b111, 0b111, 0, 0b11111) >> 13;
    sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_VL53L0X, node_id, 0, VL53L0X_CMD_DISABLE) & 0x1FFF) << 3 | 0b100;
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

    while (!vl53l0x_ctrl_global._is_initialized) { // mainから初期化命令が送られて来るまで待機
        if (HAL_CAN_GetError(hcan) == HAL_CAN_ERROR_BOF) {
            HAL_CAN_Init(hcan);
        }
        SendBytes(hcan, Make_CAN_ID(NODE_VL53L0X, node_id, 0, VL53L0X_CMD_AWAKE), &node_id, sizeof(uint8_t));
        printf("send awake cmd\n");
        HAL_Delay(1000);
    }
    printf("-------vl53l0x-------\n");
    CANVL53L0X_Print(&vl53l0x_ctrl_global);
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

HAL_StatusTypeDef PushTx8Bytes(const uint8_t device_id, const uint8_t cmd, const uint8_t *const rxData) {
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
        uint8_t device_id = buffer[readpoint].device_id;
        if (device_id != 0) {
            printf("[Process_Fifo0Msg()] device_id error\n");
        }
        switch (buffer[readpoint].cmd) {
            case VL53L0X_CMD_INIT:
                if (vl53l0x_ctrl_global._is_initialized) return HAL_ERROR;
                initialize_cmd_received = 1;
                vl53l0x_ctrl_global.need_vl53l0x_initialize = true;
                break;
            case VL53L0X_CMD_CHANGE_CTRL:
                if (!initialize_cmd_received) return HAL_ERROR;
                if (!CANVL53L0X_Change_Status(&vl53l0x_ctrl_global, VL53L0X_STATUS_CHANGE_CTRL)) return HAL_ERROR;
                vl53l0x_ctrl_global._start_change_ctrl = 1;
                vl53l0x_ctrl_global._finish_change_ctrl = 0;
                vl53l0x_ctrl_global._start_change_ctrl = 0;
                vl53l0x_ctrl_global._finish_change_ctrl = 1;
                vl53l0x_ctrl_global._is_initialized = 1; // end init
                break;
            case VL53L0X_CMD_ENABLE:
                if (!CANVL53L0X_Change_Status(&vl53l0x_ctrl_global, VL53L0X_STATUS_ENABLE)) return HAL_ERROR;
                break;
            case VL53L0X_CMD_DISABLE:
                if (!CANVL53L0X_Change_Status(&vl53l0x_ctrl_global, VL53L0X_STATUS_DISABLE)) return HAL_ERROR;
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
    if ((can_device.node_type != NODE_VL53L0X) || (can_device.node_id != node_id)) {
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

// 基板のnode idを取得する
static uint8_t _GetID(void) {
    uint8_t _node_id = ((!HAL_GPIO_ReadPin(ID1_GPIO_Port, ID1_Pin)) | (!HAL_GPIO_ReadPin(ID2_GPIO_Port, ID2_Pin) << 1) | (!HAL_GPIO_ReadPin(ID4_GPIO_Port, ID4_Pin) << 2)); // calibの仕様上node_idは3bit
    printf("id:%d\n", _node_id);
    return _node_id;
}

void CANVL53L0X_Update(VL53L0X_Ctrl_Typedef *const can_vl53l0x) { // この関数で VL53L0X に測距させてる
    if (can_vl53l0x->need_vl53l0x_initialize) {
        if (!initVL53L0X(&(can_vl53l0x->vl53l0x), 1, &hi2c1)) {
            printf("init VL53L0X error\n");
        } else {
            can_vl53l0x->need_vl53l0x_initialize = false;
        }
    }

    if (can_vl53l0x->need_vl53l0x_enable) {
        _CANVL53L0X_Enable(can_vl53l0x);
        can_vl53l0x->need_vl53l0x_enable = false;
    }

    if (can_vl53l0x->need_vl53l0x_disable) {
        _CANVL53L0X_Disable(can_vl53l0x);
        can_vl53l0x->need_vl53l0x_disable = false;
    }

    if ((can_vl53l0x->status) == VL53L0X_STATUS_ENABLE) { // VL53L0X が Enable の時の動作
        const uint16_t distance = readRangeContinuousMillimeters(&(can_vl53l0x->vl53l0x), &(can_vl53l0x->distanceStr));

        if (timeoutOccurred(&(can_vl53l0x->vl53l0x))) {
            printf("timeout\n");
            can_vl53l0x->distance = NAN;
        } else if (distance > 2000) {
            printf("too long distance\n");
            can_vl53l0x->distance = NAN;
        } else if (distance < 30) {
            printf("too short distance\n");
            can_vl53l0x->distance = NAN;
        } else {
            printf("%d mm\n", distance);
            can_vl53l0x->distance = distance * 1.0e-3f; // [m] 単位に変換
        }

    } else if ((can_vl53l0x->status) == VL53L0X_STATUS_DISABLE || (can_vl53l0x->status) == VL53L0X_STATUS_CHANGE_CTRL) {
        can_vl53l0x->distance = NAN;
    }
}

uint8_t CANVL53L0X_Send_Feedback_Main(const VL53L0X_Ctrl_Typedef *const can_vl53l0x) { // MainのマイコンにFeedbackを送信する関数（戻り値：送ったら0, 送らなかったら1）
    if (can_vl53l0x->status == VL53L0X_STATUS_INIT) return 1;

    CANVL53L0X_Feedback_Typedef tmp;
    tmp.status = can_vl53l0x->status;
    tmp.value = can_vl53l0x->distance;

    if (SendBytes(phcan, Make_CAN_ID_from_CAN_Device(&(can_vl53l0x->vl53l0x_param.device), VL53L0X_CMD_FB),
                  (uint8_t *)&tmp, sizeof(CANVL53L0X_Feedback_Typedef)) != HAL_OK) { // 送信
        Error_Handler();
    }
    return 0;
}

static bool CANVL53L0X_Change_Status(VL53L0X_Ctrl_Typedef *const can_vl53l0x, const VL53L0X_Status next_status) {
    VL53L0X_Status present_status = can_vl53l0x->status; // 現在の状態
    can_vl53l0x->incorrect_status_cmd_receive_counter_global += 1;
    switch (next_status) { // ここで状態遷移できるかを判定し, 可能なら遷移している
        case VL53L0X_STATUS_ENABLE:
            if ((present_status != VL53L0X_STATUS_CHANGE_CTRL) && (present_status != VL53L0X_STATUS_DISABLE)) return false;
            can_vl53l0x->need_vl53l0x_enable = true;
            break;
        case VL53L0X_STATUS_DISABLE:
            if (present_status != VL53L0X_STATUS_ENABLE) return false;
            can_vl53l0x->need_vl53l0x_disable = true;
            break;
        case VL53L0X_STATUS_CHANGE_CTRL:
            if ((present_status != VL53L0X_STATUS_INIT) && (present_status != VL53L0X_STATUS_CHANGE_CTRL) && (present_status != VL53L0X_STATUS_DISABLE)) return false;
            can_vl53l0x->status = VL53L0X_STATUS_CHANGE_CTRL;
            break;
        default:
            break;
    }
    can_vl53l0x->incorrect_status_cmd_receive_counter_global -= 1;
    return true;
}

static void _CANVL53L0X_Enable(VL53L0X_Ctrl_Typedef *can_vl53l0x) {
    // VL53L0X の測距を開始
    setTimeout(&(can_vl53l0x->vl53l0x), 500);
    startContinuous(&(can_vl53l0x->vl53l0x), 0);

    can_vl53l0x->status = VL53L0X_STATUS_ENABLE;
}

static void _CANVL53L0X_Disable(VL53L0X_Ctrl_Typedef *can_vl53l0x) {
    stopContinuous(&(can_vl53l0x->vl53l0x));

    can_vl53l0x->status = VL53L0X_STATUS_DISABLE;
}

void CANVL53L0X_Print(const VL53L0X_Ctrl_Typedef *const can_vl53l0x) {
    printf("{ status=");
    switch (can_vl53l0x->status) {
        case VL53L0X_STATUS_INIT:
            printf("init");
            break;
        case VL53L0X_STATUS_ENABLE:
            printf("enable");
            break;
        case VL53L0X_STATUS_DISABLE:
            printf("disable");
            break;
        case VL53L0X_STATUS_CHANGE_CTRL:
            printf("change ctrl");
            break;
        default:
            break;
    }
    printf(", distance=%f, ", can_vl53l0x->distance);
    printf("er_cmd=%d ", (int)can_vl53l0x->incorrect_status_cmd_receive_counter_global);
    printf("}\n");
}

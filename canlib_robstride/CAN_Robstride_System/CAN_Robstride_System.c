/*
 * CAN_Robstride_System.c
 */

// Includes --------------------------------

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <CAN_Robstride_Def.h>
#include <CAN_Robstride_System.h>
#include <robstride_constant.h>

// Variables --------------------------------

CAN_HandleTypeDef *_robstride_phcan_global; // 変更しない事
CAN_RingBuf_Robstride robstride_can_buf_ring1 = { 0 };
robstride_feedback_data_raw _robstride_feedback_data_raw_global[129];
Robstride_FeedbackData robstride_fb_data_global[129];

// Private Function Prototypes --------------------------------

static float uint_to_float(uint16_t x, float x_min, float x_max);
static HAL_StatusTypeDef _Robstride_PushTx8Bytes(CAN_RingBuf_Robstride *p_can_ring, uint32_t ExtId, const uint8_t *bytes, uint32_t size);
static HAL_StatusTypeDef _Robstride_PopSendTx8Bytes(CAN_HandleTypeDef *phcan, CAN_RingBuf_Robstride *p_can_ring);
static void Robstride_set_fb_data_raw(uint32_t ExtID, const uint8_t rxData[], uint8_t device_id);

// Functions --------------------------------

static float uint_to_float(const uint16_t x, const float x_min, const float x_max) {
    const uint16_t type_max = 0xFFFF;
    const float span = x_max - x_min;
    return (float)x / type_max * span + x_min;
}

static HAL_StatusTypeDef _Robstride_PushTx8Bytes(CAN_RingBuf_Robstride *const p_can_ring, const uint32_t ExtId, const uint8_t *const bytes, const uint32_t size) {
    p_can_ring->buffer[p_can_ring->write_point].DLC = size;
    p_can_ring->buffer[p_can_ring->write_point].ExtId = ExtId;
    for (uint8_t i = 0; i < size; i++) p_can_ring->buffer[p_can_ring->write_point].bytes[i] = bytes[i];

    if (p_can_ring->is_full == 1) {
        p_can_ring->read_point = ((p_can_ring->read_point) + 1) & (CAN_TXBUFFER_SIZE - 1);
    }
    p_can_ring->write_point = ((p_can_ring->write_point) + 1) & (CAN_TXBUFFER_SIZE - 1);

    if (p_can_ring->write_point == p_can_ring->read_point) {
        p_can_ring->is_full = 1;
    }
    return HAL_OK;
}

static HAL_StatusTypeDef _Robstride_PopSendTx8Bytes(CAN_HandleTypeDef *const phcan, CAN_RingBuf_Robstride *const p_can_ring) {
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;

    txHeader.RTR = CAN_RTR_DATA;
    txHeader.IDE = CAN_ID_EXT; // 拡張フォーマット
    txHeader.TransmitGlobalTime = DISABLE;
    while (HAL_CAN_GetTxMailboxesFreeLevel(phcan) > 0) {
        if ((p_can_ring->is_full == 0) && (p_can_ring->read_point == p_can_ring->write_point)) break;

        txHeader.DLC = p_can_ring->buffer[p_can_ring->read_point].DLC;
        txHeader.StdId = 0; // 標準IDは使わない
        txHeader.ExtId = p_can_ring->buffer[p_can_ring->read_point].ExtId;

        HAL_StatusTypeDef ret = HAL_CAN_AddTxMessage(phcan, &txHeader, p_can_ring->buffer[p_can_ring->read_point].bytes, &txMailbox);
        if (ret != HAL_OK) return ret;
        p_can_ring->read_point = ((p_can_ring->read_point) + 1) & (CAN_TXBUFFER_SIZE - 1);
        p_can_ring->is_full = 0;
    }
    return HAL_OK;
}

void Robstride_RequestReadParameter(Robstride_DeviceInfo *const device_info, const uint16_t address) {
    uint8_t can_data[8];
    can_data[0] = address & 0x00FF;
    can_data[1] = address >> 8;
    uint16_t option = 0x00 << 8 | device_info->master_id;
    Robstride_SendBytes(device_info->phcan, device_info->device_id, CMD_RAM_READ, option, (uint8_t *)can_data, sizeof(can_data));
}

void Robstride_WriteFloatData(Robstride_DeviceInfo *const device_info, const uint16_t address, const float data) {
    uint8_t can_data[8] = { 0x00 };
    can_data[0] = address & 0x00FF;
    can_data[1] = address >> 8;
    memcpy(&can_data[4], &data, 4);
    Robstride_SendBytes(device_info->phcan, device_info->device_id, CMD_RAM_WRITE, device_info->master_id, (uint8_t *)can_data, sizeof(can_data));
}

void Robstride_WriteIntData(Robstride_DeviceInfo *const device_info, const uint16_t address, const int data) {
    uint8_t can_data[8];
    can_data[0] = address & 0x00FF;
    can_data[1] = address >> 8;
    can_data[4] = (uint8_t)data;
    Robstride_SendBytes(device_info->phcan, device_info->device_id, CMD_RAM_WRITE, device_info->master_id, (uint8_t *)can_data, sizeof(can_data));
}

HAL_StatusTypeDef Robstride_SendBytes(CAN_HandleTypeDef *const phcan, const uint8_t motor_id, const uint8_t cmd_id, const uint16_t option, const uint8_t *const bytes, const uint32_t size) { // 命令を送信する関数
    const uint32_t quotient = size / 8;
    const uint32_t remainder = size - (8 * quotient);
    HAL_StatusTypeDef ret;
    const uint32_t ExtId = cmd_id << 24 | option << 8 | motor_id;
    for (uint8_t i = 0; i < quotient; i++) {
        ret = _Robstride_PushTx8Bytes(&robstride_can_buf_ring1, ExtId, bytes + i * 8, 8);
        if (ret != HAL_OK) {
            Error_Handler();
            return ret;
        }
    }

    if (remainder != 0) {
        ret = _Robstride_PushTx8Bytes(&robstride_can_buf_ring1, ExtId, bytes + quotient * 8, remainder);
        if (ret != HAL_OK) {
            Error_Handler();
            return ret;
        }
    }
    ret = _Robstride_PopSendTx8Bytes(phcan, &robstride_can_buf_ring1);
    if (ret != HAL_OK) {
        Error_Handler();
        return ret;
    }
    return HAL_OK;
}

void Robstride_WhenTxMailboxCompleteCallbackCalled(CAN_HandleTypeDef *const phcan) {
    if (_robstride_phcan_global != phcan) return;
    _Robstride_PopSendTx8Bytes(phcan, &robstride_can_buf_ring1);
}

void Robstride_WhenTxMailboxAbortCallbackCalled(CAN_HandleTypeDef *const phcan) {
    if (_robstride_phcan_global != phcan) return;
    _Robstride_PopSendTx8Bytes(phcan, &robstride_can_buf_ring1);
}

void Get_Robstride_MCUID(const uint8_t rxData[], uint8_t device_id) {
    memcpy(&_robstride_feedback_data_raw_global[device_id].mcu_id, &rxData[0], 8);
    robstride_fb_data_global[device_id].mcu_id = _robstride_feedback_data_raw_global[device_id].mcu_id;
    return;
}

void Robstride_SetCANID(Robstride_DeviceInfo *const device_info, const uint8_t new_id) {
    const uint8_t can_data[8] = { 0x00 };
    const uint16_t option = new_id << 8 | device_info->master_id;
    Robstride_SendBytes(device_info->phcan, device_info->device_id, CMD_CHANGE_CAN_ID, option, (uint8_t *)can_data, sizeof(can_data));
    device_info->device_id = new_id;
}

static void Robstride_set_fb_data_raw(const uint32_t ExtID, const uint8_t rxData[], const uint8_t device_id) {
    _robstride_feedback_data_raw_global[device_id]._get_counter += 1;
    if (_robstride_feedback_data_raw_global[device_id]._get_counter > 128) {
        _robstride_feedback_data_raw_global[device_id]._get_counter = 3; // overflow対策
    }
    _robstride_feedback_data_raw_global[device_id].pos = rxData[1] | rxData[0] << 8;
    _robstride_feedback_data_raw_global[device_id].vel = rxData[3] | rxData[2] << 8;
    _robstride_feedback_data_raw_global[device_id].torque = rxData[5] | rxData[4] << 8;
    _robstride_feedback_data_raw_global[device_id].temp = rxData[7] | rxData[6] << 8;

    robstride_fb_data_global[device_id].device_id = device_id;
    robstride_fb_data_global[device_id].get_flag = (_robstride_feedback_data_raw_global[device_id]._get_counter > 2);
    const uint8_t mode_status = (ExtID >> 22) & 0x03; // bit22と23を抽出
    robstride_fb_data_global[device_id].mode_status = mode_status;

    switch (robstride_fb_data_global[device_id].device) {
        case Robstride_02:
            robstride_fb_data_global[device_id].position = uint_to_float(_robstride_feedback_data_raw_global[device_id].pos, P_MIN_ROBSTRIDE02, P_MAX_ROBSTRIDE02) * robstride_fb_data_global[device_id].quant_per_rot * robstride_fb_data_global[device_id].plus_minus;
            robstride_fb_data_global[device_id].position += robstride_fb_data_global[device_id].offset_pos;
            robstride_fb_data_global[device_id].velocity = uint_to_float(_robstride_feedback_data_raw_global[device_id].vel, V_MIN_ROBSTRIDE02, V_MAX_ROBSTRIDE02) * robstride_fb_data_global[device_id].quant_per_rot * robstride_fb_data_global[device_id].plus_minus;
            robstride_fb_data_global[device_id].torque = uint_to_float(_robstride_feedback_data_raw_global[device_id].torque, T_MIN_ROBSTRIDE02, T_MAX_ROBSTRIDE02);
            break;
        case Robstride_04:
            robstride_fb_data_global[device_id].position = uint_to_float(_robstride_feedback_data_raw_global[device_id].pos, P_MIN_ROBSTRIDE04, P_MAX_ROBSTRIDE04) * robstride_fb_data_global[device_id].quant_per_rot * robstride_fb_data_global[device_id].plus_minus;
            robstride_fb_data_global[device_id].position += robstride_fb_data_global[device_id].offset_pos;
            robstride_fb_data_global[device_id].velocity = uint_to_float(_robstride_feedback_data_raw_global[device_id].vel, V_MIN_ROBSTRIDE04, V_MAX_ROBSTRIDE04) * robstride_fb_data_global[device_id].quant_per_rot * robstride_fb_data_global[device_id].plus_minus;
            robstride_fb_data_global[device_id].torque = uint_to_float(_robstride_feedback_data_raw_global[device_id].torque, T_MIN_ROBSTRIDE04, T_MAX_ROBSTRIDE04);
            break;

        case Robstride_05_Edu:
            robstride_fb_data_global[device_id].position = uint_to_float(_robstride_feedback_data_raw_global[device_id].pos, P_MIN_ROBSTRIDE05, P_MAX_ROBSTRIDE05) * robstride_fb_data_global[device_id].quant_per_rot * robstride_fb_data_global[device_id].plus_minus;
            robstride_fb_data_global[device_id].position += robstride_fb_data_global[device_id].offset_pos;
            robstride_fb_data_global[device_id].velocity = uint_to_float(_robstride_feedback_data_raw_global[device_id].vel, V_MIN_ROBSTRIDE05, V_MAX_ROBSTRIDE05) * robstride_fb_data_global[device_id].quant_per_rot * robstride_fb_data_global[device_id].plus_minus;
            robstride_fb_data_global[device_id].torque = uint_to_float(_robstride_feedback_data_raw_global[device_id].torque, T_MIN_ROBSTRIDE05, T_MAX_ROBSTRIDE05);
            break;
        default:
            break;
    }
    //    printf("id: %u, pos: %f, vel: %f, torque: %f\n\r",
    //           device_id,
    //           robstride_fb_data_global[device_id].position,
    //           robstride_fb_data_global[device_id].velocity,
    //           robstride_fb_data_global[device_id].torque);
    robstride_fb_data_global[device_id].temperature = (int)((float)(_robstride_feedback_data_raw_global[device_id].temp) / 10.0);
}

void Robstride_WhenCANRxFifo0MsgPending(CAN_HandleTypeDef *const phcan) {
    if (_robstride_phcan_global != phcan) return;
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    if (HAL_CAN_GetRxMessage(phcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
        return;
    }
    if (rxHeader.IDE != CAN_ID_EXT) return;

    uint8_t motor_id = 0;
    uint32_t ExtId = rxHeader.ExtId;
    if (ExtId >= 0x00FE && ExtId <= 0x7FFE) {
        motor_id = (uint8_t)(ExtId >> 8);
        //    printf("response1 from motor from %d\n\r", (int)motor_id);
        Get_Robstride_MCUID(rxData, motor_id);
    } else if (ExtId >= 0x02000000 && ExtId <= 0x02C07F7F) {
        // uint32_t master_id = (uint8_t)(ExtId & 0xFF);
        motor_id = (uint8_t)((ExtId >> 8) & 0xFF);
        Robstride_set_fb_data_raw(ExtId, rxData, motor_id);
        //    printf("response2 from motor from %d to %d\n\r", (int)motor_id, (int)master_id);
    } else if (ExtId >= 0x11000000 && ExtId <= 0x11007F7F) {
        // uint32_t master_id = (uint8_t)(ExtId & 0xFF);
        motor_id = (uint8_t)((ExtId >> 8) & 0xFF);
        Robstride_ProcessParameter(rxData, motor_id);
        // printf("response3 from motor from %d to %d\n\r", (int)motor_id, (int)master_id);
    } else if (ExtId >= 0x15000000 && ExtId <= 0x15007F7F) {
        // uint32_t master_id = (uint8_t)((ExtId >> 8)) & 0xFF;
        motor_id = (uint8_t)(ExtId & 0xFF);
        Robstride_ProcessFault(rxData, motor_id);
        // printf("response4 from motor from %d to %d\n\r", (int)motor_id, (int)master_id);
    }
    // printf("got response from %d\n\r", (int)motor_id);
}

void Robstride_WhenCANRxFifo1MsgPending(CAN_HandleTypeDef *const phcan) {
    if (_robstride_phcan_global != phcan) return;
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    if (HAL_CAN_GetRxMessage(phcan, CAN_RX_FIFO1, &rxHeader, rxData) != HAL_OK) {
        return;
    }
    if (rxHeader.IDE != CAN_ID_EXT) return;

    uint8_t motor_id = 0;
    uint32_t ExtId = rxHeader.ExtId;
    if (ExtId >= 0x00FE && ExtId <= 0x7FFE) {
        motor_id = (uint8_t)(ExtId >> 8);
        // printf("response1 from motor from %d\n\r", (int)motor_id);
        Get_Robstride_MCUID(rxData, motor_id);
    } else if (ExtId >= 0x02000000 && ExtId <= 0x02C07F7F) {
        // uint32_t master_id = (uint8_t)(ExtId & 0xFF);
        motor_id = (uint8_t)((ExtId >> 8) & 0xFF);
        Robstride_set_fb_data_raw(ExtId, rxData, motor_id);
        // printf("response2 from motor from %d to %d\n\r", (int)motor_id, (int)master_id);
    } else if (ExtId >= 0x11000000 && ExtId <= 0x11007F7F) {
        // uint32_t master_id = (uint8_t)(ExtId & 0xFF);
        motor_id = (uint8_t)((ExtId >> 8) & 0xFF);
        Robstride_ProcessParameter(rxData, motor_id);
        // printf("response3 from motor from %d to %d\n\r", (int)motor_id, (int)master_id);
    } else if (ExtId >= 0x15000000 && ExtId <= 0x15007F7F) {
        // uint32_t master_id = (uint8_t)((ExtId >> 8)) & 0xFF;
        motor_id = (uint8_t)(ExtId & 0xFF);
        Robstride_ProcessFault(rxData, motor_id);
        // printf("response4 from motor from %d to %d\n\r", (int)motor_id, (int)master_id);
    }
    // printf("got response from %d\n\r", (int)motor_id);
}

void Init_Robstride_CAN_System(CAN_HandleTypeDef *const phcan) { // CAN初期化
    _robstride_phcan_global = phcan;
    CAN_FilterTypeDef sFilterConfig;

    uint32_t FilterID, FilterMaskID;

    // フィルタバンク設定
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (phcan->Instance == CAN1 || phcan->Instance == CAN3) {
        // CAN1 or 3 使用時のフィルタを設定

        // Get device ID (Communication type 0) のフィルタを FIFO 0 に設定
        FilterID = (0x00000000 | 0xFE) << 3;
        FilterMaskID = (0x1F000000 | 0xFF) << 3;
        sFilterConfig.FilterBank = 2;
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
        sFilterConfig.FilterIdHigh = FilterID >> 16;
        sFilterConfig.FilterMaskIdHigh = FilterMaskID >> 16;
        sFilterConfig.FilterIdLow = FilterID;
        sFilterConfig.FilterMaskIdLow = FilterMaskID;
        if (HAL_CAN_ConfigFilter(phcan, &sFilterConfig) != HAL_OK) {
            Error_Handler();
        }

        // Response motor feedback (Communication type 2) のフィルタを FIFO 1 に設定
        FilterID = 0x02000000 << 3;
        FilterMaskID = 0x1F000000 << 3;
        sFilterConfig.FilterBank = 3;
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
        sFilterConfig.FilterIdHigh = FilterID >> 16;
        sFilterConfig.FilterMaskIdHigh = FilterMaskID >> 16;
        sFilterConfig.FilterIdLow = FilterID;
        sFilterConfig.FilterMaskIdLow = FilterMaskID;
        if (HAL_CAN_ConfigFilter(phcan, &sFilterConfig) != HAL_OK) {
            Error_Handler();
        }

        // Single parameter read (Communication type 17) のフィルタを FIFO 0 に設定
        FilterID = 0x11000000 << 3;
        FilterMaskID = 0x1F000000 << 3;
        sFilterConfig.FilterBank = 4;
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
        sFilterConfig.FilterIdHigh = FilterID >> 16;
        sFilterConfig.FilterMaskIdHigh = FilterMaskID >> 16;
        sFilterConfig.FilterIdLow = FilterID;
        sFilterConfig.FilterMaskIdLow = FilterMaskID;
        if (HAL_CAN_ConfigFilter(phcan, &sFilterConfig) != HAL_OK) {
            Error_Handler();
        }

        // Fault feedback frame (Communication type 21) のフィルタを FIFO 0 に設定
        FilterID = 0x15000000 << 3;
        FilterMaskID = 0x1F000000 << 3;
        sFilterConfig.FilterBank = 5;
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
        sFilterConfig.FilterIdHigh = FilterID >> 16;
        sFilterConfig.FilterMaskIdHigh = FilterMaskID >> 16;
        sFilterConfig.FilterIdLow = FilterID;
        sFilterConfig.FilterMaskIdLow = FilterMaskID;
        if (HAL_CAN_ConfigFilter(phcan, &sFilterConfig) != HAL_OK) {
            Error_Handler();
        }

    } else if (phcan->Instance == CAN2) {
        // CAN2 使用時のフィルタを設定

        // Get device ID (Communication type 0) のフィルタを FIFO 0 に設定
        FilterID = (0x00000000 | 0xFE) << 3;
        FilterMaskID = (0x1F000000 | 0xFF) << 3;
        sFilterConfig.FilterBank = 15;
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
        sFilterConfig.FilterIdHigh = FilterID >> 16;
        sFilterConfig.FilterMaskIdHigh = FilterMaskID >> 16;
        sFilterConfig.FilterIdLow = FilterID;
        sFilterConfig.FilterMaskIdLow = FilterMaskID;
        if (HAL_CAN_ConfigFilter(phcan, &sFilterConfig) != HAL_OK) {
            Error_Handler();
        }

        // Response motor feedback (Communication type 2) のフィルタを FIFO 1 に設定
        FilterID = 0x02000000 << 3;
        FilterMaskID = 0x1F000000 << 3;
        sFilterConfig.FilterBank = 16;
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
        sFilterConfig.FilterIdHigh = FilterID >> 16;
        sFilterConfig.FilterMaskIdHigh = FilterMaskID >> 16;
        sFilterConfig.FilterIdLow = FilterID;
        sFilterConfig.FilterMaskIdLow = FilterMaskID;
        if (HAL_CAN_ConfigFilter(phcan, &sFilterConfig) != HAL_OK) {
            Error_Handler();
        }

        // Single parameter read (Communication type 17) のフィルタを FIFO 0 に設定
        FilterID = 0x11000000 << 3;
        FilterMaskID = 0x1F000000 << 3;
        sFilterConfig.FilterBank = 17;
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
        sFilterConfig.FilterIdHigh = FilterID >> 16;
        sFilterConfig.FilterMaskIdHigh = FilterMaskID >> 16;
        sFilterConfig.FilterIdLow = FilterID;
        sFilterConfig.FilterMaskIdLow = FilterMaskID;
        if (HAL_CAN_ConfigFilter(phcan, &sFilterConfig) != HAL_OK) {
            Error_Handler();
        }

        // Fault feedback frame (Communication type 21) のフィルタを FIFO 0 に設定
        FilterID = 0x15000000 << 3;
        FilterMaskID = 0x1F000000 << 3;
        sFilterConfig.FilterBank = 18;
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
        sFilterConfig.FilterIdHigh = FilterID >> 16;
        sFilterConfig.FilterMaskIdHigh = FilterMaskID >> 16;
        sFilterConfig.FilterIdLow = FilterID;
        sFilterConfig.FilterMaskIdLow = FilterMaskID;
        if (HAL_CAN_ConfigFilter(phcan, &sFilterConfig) != HAL_OK) {
            Error_Handler();
        }

    } else {
        printf("CAN Instance Error\n");
        Error_Handler();
    }

    if (HAL_CAN_Start(phcan) != HAL_OK) {
        printf(" -> Start Error CAN_Robstride\n");
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(phcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        printf(" -> FIFO0 CAN_Activation error\n\r");
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(phcan, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) {
        printf(" -> FIFO1 CAN_Activation error1\n\r");
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(phcan, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK) {
        printf(" -> CAN_Activation error2\n\r");
        Error_Handler();
    }

    for (uint8_t i = 0; i < 129; i++) { // init fb_data_raw
        _robstride_feedback_data_raw_global[i].pos = 0;
        _robstride_feedback_data_raw_global[i]._rot_num = 0;
        _robstride_feedback_data_raw_global[i].vel = 0;
        _robstride_feedback_data_raw_global[i].torque = 0;
        _robstride_feedback_data_raw_global[i].temp = 0;
        _robstride_feedback_data_raw_global[i]._get_counter = 0;
        robstride_fb_data_global[i].position = 0;
        robstride_fb_data_global[i].velocity = 0;
        robstride_fb_data_global[i].torque = 0;
        robstride_fb_data_global[i].temperature = 0;
        robstride_fb_data_global[i].current = 0;
    }
}

/**
 * @brief モーターから複数のパラメータを個別に読み出し、フィードバックデータを更新します。
 * @note 【非推奨】この関数は、モーターに対してパラメータ読み出しコマンド（Type 17）を個別に送信します。
 * しかし、以下の制御コマンドを送信した際には、モーターはその応答として標準フィードバック（Type 2）を自動的に返信します。
 * - Type 1: 操作制御モード命令 (Robstride_SetTargetなど)
 * - Type 3: モーター運転開始 (Robstride_ControlEnable)
 * - Type 4: モーター運転停止 (Robstride_ControlDisable)
 * - Type 6: ゼロ点設定
 * - Type 18: パラメータ書き込み
 * - Type 22: データ保存
 *
 * したがって、制御ループ内で上記のコマンドを周期的に送信している場合、
 * この関数を別途呼び出す必要はありません。
 * むしろ、異なる2系統のフィードバックを非同期に取得することになり、
 *　Posデータが１から２度ずれるため、使用は推奨されません。
 * 代わりに`Read_Robstride_FeedbackData`を使用してください。
 * また、標準フィードバックで更新されるデータは位置、速度、トルクなので、これら以外のパラメーターが必要な場合は、
 * それぞれで個別に読み出しコマンドを送信してください。
 * @param device_info Robstrideデバイス情報構造体へのポインタ
 * @return Robstride_FeedbackData 更新されたフィードバックデータ
 */
Robstride_FeedbackData Get_Robstride_FeedbackData(Robstride_DeviceInfo *const device_info) {
    Robstride_RequestReadParameter(device_info, ADDR_MECH_POS);
    Robstride_RequestReadParameter(device_info, ADDR_MECH_VEL);
    Robstride_RequestReadParameter(device_info, ADDR_IQF);
    uint8_t device_id = device_info->device_id;
    robstride_fb_data_global[device_id].device_id = device_id;
    return robstride_fb_data_global[device_id];
}
Robstride_FeedbackData Read_Robstride_FeedbackData(Robstride_DeviceInfo *const device_info) {
    uint8_t device_id = device_info->device_id;
    robstride_fb_data_global[device_id].device_id = device_id;
    return robstride_fb_data_global[device_id];
}

void Robstride_ProcessParameter(const uint8_t rxData[], const uint8_t device_id) {
    const uint16_t address = rxData[0] | rxData[1] << 8;
    uint8_t uint8_data;
    // int16_t int16_data;
    float float_data;
    uint16_t uint16_data;
    uint32_t uint32_data;

    switch (address) {
        case ADDR_RUN_MODE: // 0x7005
            memcpy(&uint8_data, &rxData[4], sizeof(uint8_data));
            robstride_fb_data_global[device_id].run_mode = uint8_data;
            break;
        case ADDR_IQ_REF: // 0x7006
            memcpy(&float_data, &rxData[4], sizeof(float_data));
            robstride_fb_data_global[device_id].iq_ref = float_data;
            break;
        case ADDR_SPEED_REF: // 0x700A
            memcpy(&float_data, &rxData[4], sizeof(float_data));
            robstride_fb_data_global[device_id].spd_ref = float_data;
            break;
        case ADDR_LIMIT_TORQUE: // 0x700B
            memcpy(&float_data, &rxData[4], sizeof(float_data));
            robstride_fb_data_global[device_id].limit_torque = float_data;
            break;
        case ADDR_CURRENT_KP: // 0x7010
            memcpy(&float_data, &rxData[4], sizeof(float_data));
            robstride_fb_data_global[device_id].cur_kp = float_data;
            break;
        case ADDR_CURRENT_KI: // 0x7011
            memcpy(&float_data, &rxData[4], sizeof(float_data));
            robstride_fb_data_global[device_id].cur_ki = float_data;
            break;
        case ADDR_CURRENT_FILTER_GAIN: // 0x7014
            memcpy(&float_data, &rxData[4], sizeof(float_data));
            robstride_fb_data_global[device_id].cur_filt_gain = float_data;
            break;
        case ADDR_LOC_REF: // 0x7016
            memcpy(&float_data, &rxData[4], sizeof(float_data));
            robstride_fb_data_global[device_id].loc_ref = float_data;
            break;
        case ADDR_LIMIT_SPEED: // 0x7017
            memcpy(&float_data, &rxData[4], sizeof(float_data));
            robstride_fb_data_global[device_id].limit_spd = float_data;
            break;
        case ADDR_LIMIT_CURRENT: // 0x7018
            memcpy(&float_data, &rxData[4], sizeof(float_data));
            robstride_fb_data_global[device_id].limit_cur = float_data;
            break;
        case ADDR_MECH_POS:
            memcpy(&float_data, &rxData[4], sizeof(float_data));
            robstride_fb_data_global[device_id].position = float_data * robstride_fb_data_global[device_id].quant_per_rot * robstride_fb_data_global[device_id].plus_minus;
            robstride_fb_data_global[device_id].position += robstride_fb_data_global[device_id].offset_pos;
            // printf("motor%d: pos %f\n\r", (int)device_id, robstride_fb_data_global[device_id].position);
            _robstride_feedback_data_raw_global[device_id]._get_counter += 1;
            if (_robstride_feedback_data_raw_global[device_id]._get_counter > 128) {
                _robstride_feedback_data_raw_global[device_id]._get_counter = 3; // overflow対策
            }
            robstride_fb_data_global[device_id].get_flag = (_robstride_feedback_data_raw_global[device_id]._get_counter > 2);
            break;
        case ADDR_IQF:
            memcpy(&float_data, &rxData[4], sizeof(float_data));
            robstride_fb_data_global[device_id].current = float_data * robstride_fb_data_global[device_id].plus_minus;
            _robstride_feedback_data_raw_global[device_id]._get_counter += 1;
            // printf("motor%d: current %f\n\r", (int)device_id, robstride_fb_data_global[device_id].current);
            if (_robstride_feedback_data_raw_global[device_id]._get_counter > 128) {
                _robstride_feedback_data_raw_global[device_id]._get_counter = 3; // overflow対策
            }
            robstride_fb_data_global[device_id].get_flag = (_robstride_feedback_data_raw_global[device_id]._get_counter > 2);
            break;
        case ADDR_MECH_VEL:
            memcpy(&float_data, &rxData[4], sizeof(float_data));
            robstride_fb_data_global[device_id].velocity = float_data * robstride_fb_data_global[device_id].quant_per_rot * robstride_fb_data_global[device_id].plus_minus;
            // printf("motor%d: vel %f\n\r", (int)device_id, robstride_fb_data_global[device_id].velocity);
            _robstride_feedback_data_raw_global[device_id]._get_counter += 1;
            if (_robstride_feedback_data_raw_global[device_id]._get_counter > 128) {
                _robstride_feedback_data_raw_global[device_id]._get_counter = 3; // overflow対策
            }
            robstride_fb_data_global[device_id].get_flag = (_robstride_feedback_data_raw_global[device_id]._get_counter > 2);
            break;
        case ADDR_VBUS: // 0x701C
            memcpy(&float_data, &rxData[4], sizeof(float_data));
            robstride_fb_data_global[device_id].vbus = float_data;
            break;
        case ADDR_LOC_KP: // 0x701E
            memcpy(&float_data, &rxData[4], sizeof(float_data));
            robstride_fb_data_global[device_id].loc_kp = float_data;
            break;
        case ADDR_SPD_KP: // 0x701F
            memcpy(&float_data, &rxData[4], sizeof(float_data));
            robstride_fb_data_global[device_id].spd_kp = float_data;
            break;
        case ADDR_SPD_KI: // 0x7020
            memcpy(&float_data, &rxData[4], sizeof(float_data));
            robstride_fb_data_global[device_id].spd_ki = float_data;
            break;
        case ADDR_SPD_FILTER_GAIN: // 0x7021
            memcpy(&float_data, &rxData[4], sizeof(float_data));
            robstride_fb_data_global[device_id].spd_filt_gain = float_data;
            break;
        case ADDR_ACC_RAD: // 0x7022
            memcpy(&float_data, &rxData[4], sizeof(float_data));
            robstride_fb_data_global[device_id].acc_rad = float_data;
            break;
        case ADDR_VEL_MAX_PP: // 0x7024
            memcpy(&float_data, &rxData[4], sizeof(float_data));
            robstride_fb_data_global[device_id].vel_max_pp = float_data;
            break;
        case ADDR_ACC_SET_PP: // 0x7025
            memcpy(&float_data, &rxData[4], sizeof(float_data));
            robstride_fb_data_global[device_id].acc_set_pp = float_data;
            break;
        case ADDR_EPSCAN_TIME: // 0x7026
            memcpy(&uint16_data, &rxData[4], sizeof(uint16_data));
            robstride_fb_data_global[device_id].epscan_time = uint16_data;
            break;
        case ADDR_CAN_TIMEOUT: // 0x7028
            memcpy(&uint32_data, &rxData[4], sizeof(uint32_data));
            robstride_fb_data_global[device_id].can_timeout = uint32_data;
            break;
        case ADDR_ZERO_STA: // 0x7029
            memcpy(&uint8_data, &rxData[4], sizeof(uint8_data));
            robstride_fb_data_global[device_id].zero_sta = uint8_data;
            break;
        case ADDR_ADD_OFFSET: // 0x702B
            memcpy(&float_data, &rxData[4], sizeof(float_data));
            robstride_fb_data_global[device_id].add_offset = float_data;
            break;
        default:
            break;
    }
}

void Robstride_ProcessFault(const uint8_t rxData[], const uint8_t device_id) {
    if ((rxData[0] & 0x01) != 0) {
        printf("motor%d:Motor over temperature fault\n\r", (int)device_id);
    }
    if ((rxData[0] & 0x02) != 0) {
        printf("motor%d:Driver chip failure\n\r", (int)device_id);
    }
    if ((rxData[0] & 0x04) != 0) {
        printf("motor%d:Undervoltage fault\n\r", (int)device_id);
    }
    if ((rxData[0] & 0x08) != 0) {
        printf("motor%d:Overvoltage fault\n\r", (int)device_id);
    }
    if ((rxData[0] & 0x10) != 0) {
        printf("motor%d:B-phase current sampling overcurrent\n\r", (int)device_id);
    }
    if ((rxData[0] & 0x20) != 0) {
        printf("motor%d:C-phase current sampling overcurrent\n\r", (int)device_id);
    }
    if ((rxData[0] & 0x80) != 0) {
        printf("motor%d:Encoder not calibrated\n\r", (int)device_id);
    }
    if ((rxData[0] & 0x100) != 0) {
        printf("motor%d:Overload fault\n\r", (int)device_id);
    }
    if ((rxData[0] & 0x10000) != 0) {
        printf("motor%d:A-phase current sampling overcurrent\n\r", (int)device_id);
    }
    if ((rxData[4] & 0x01) != 0) {
        printf("motor%d:Motor over temperature warning\n\r", (int)device_id);
    }
}

void Robstride_fb_init(Robstride_DeviceInfo *const device_info) {
    if (device_info->ctrl_param.rotation == ROBSTRIDE_ROT_CW) {
        robstride_fb_data_global[device_info->device_id].plus_minus = -1;
    } else {
        robstride_fb_data_global[device_info->device_id].plus_minus = 1;
    }
    robstride_fb_data_global[device_info->device_id].quant_per_rot = device_info->ctrl_param.quant_per_rot;
    robstride_fb_data_global[device_info->device_id].offset_pos = device_info->ctrl_param.offset_pos;
    robstride_fb_data_global[device_info->device_id].device = device_info->device;
}

/*
 * CAN_C620_System.c
 *
 *  Created on: 7 8, 2023
 *      Author: Emile
 */


#include <stdio.h>
#include <stdint.h>
#include "CAN_RoboMas_Def.h"
#include "CAN_RoboMas_System.h"


#define ROBOMAS_CAN_TXBUFFER_SIZE    (512)
#define ROBOMAS_FEEDBACK_READY_COUNT (50U)

typedef struct {
    uint32_t StdId; // 18bit
    uint32_t DLC;
    uint8_t bytes[8];
} CANTxBuf_StdID;

typedef struct {
    CANTxBuf_StdID buffer[ROBOMAS_CAN_TXBUFFER_SIZE];
    uint32_t read_point;
    uint32_t write_point;
    uint8_t is_full;
} CANRingBuf_StdID;


CAN_HandleTypeDef *_robomas_phcan_global; // 変更しない事
CANRingBuf_StdID _robomas_can_buf_ring1 = { 0 };
robomas_feedback_data_raw _robomas_feedback_data_raw_global[9];


HAL_StatusTypeDef _RoboMas_PushTx8Bytes(CANRingBuf_StdID* p_can_ring, uint32_t StdId, uint8_t* bytes, uint32_t size) {
    p_can_ring->buffer[p_can_ring->write_point].DLC = size;
    p_can_ring->buffer[p_can_ring->write_point].StdId = StdId;
    for (uint8_t i = 0; i < size; i++)p_can_ring->buffer[p_can_ring->write_point].bytes[i] = bytes[i];

    if (p_can_ring->is_full == 1) {
        p_can_ring->read_point = ((p_can_ring->read_point) + 1) & (ROBOMAS_CAN_TXBUFFER_SIZE - 1);
    }
    p_can_ring->write_point = ((p_can_ring->write_point) + 1) & (ROBOMAS_CAN_TXBUFFER_SIZE - 1);

    if (p_can_ring->write_point == p_can_ring->read_point) {
        p_can_ring->is_full = 1;
    }
    return HAL_OK;
}

HAL_StatusTypeDef _RoboMas_PopSendTx8Bytes(CAN_HandleTypeDef* phcan, CANRingBuf_StdID* p_can_ring) {
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;

    txHeader.RTR = CAN_RTR_DATA;
    txHeader.IDE = CAN_ID_STD;
    txHeader.TransmitGlobalTime = DISABLE;
    while (HAL_CAN_GetTxMailboxesFreeLevel(phcan) > 0) {
        if ((p_can_ring->is_full == 0) && (p_can_ring->read_point == p_can_ring->write_point))break;

        txHeader.DLC = p_can_ring->buffer[p_can_ring->read_point].DLC;
        txHeader.StdId = p_can_ring->buffer[p_can_ring->read_point].StdId;
        txHeader.ExtId = 0;

        HAL_StatusTypeDef ret = HAL_CAN_AddTxMessage(phcan, &txHeader, p_can_ring->buffer[p_can_ring->read_point].bytes,
                                                     &txMailbox);
        if (ret != HAL_OK)return ret;
        p_can_ring->read_point = ((p_can_ring->read_point) + 1) & (ROBOMAS_CAN_TXBUFFER_SIZE - 1);
        p_can_ring->is_full = 0;
    }
    return HAL_OK;
}


HAL_StatusTypeDef RoboMas_SendBytes(CAN_HandleTypeDef *phcan, uint32_t StdId, uint8_t *bytes, uint32_t size) { // 命令を送信する関数
    uint32_t quotient = size / 8;
    uint32_t remainder = size - (8 * quotient);
    HAL_StatusTypeDef ret;

    for (uint8_t i = 0; i < quotient; i++) {
        ret = _RoboMas_PushTx8Bytes(&_robomas_can_buf_ring1, StdId, bytes + i * 8, 8);
        if (ret != HAL_OK) {
            Error_Handler();
            return ret;
        }
    }

    if (remainder != 0) {
        ret = _RoboMas_PushTx8Bytes(&_robomas_can_buf_ring1, StdId, bytes + quotient * 8, remainder);
        if (ret != HAL_OK) {
            Error_Handler();
            return ret;
        }
    }
    ret = _RoboMas_PopSendTx8Bytes(phcan, &_robomas_can_buf_ring1);
    if (ret != HAL_OK) {
        Error_Handler();
        return ret;
    }
    return HAL_OK;
}


void RoboMas_WhenTxMailboxCompleteCallbackCalled(CAN_HandleTypeDef *phcan) {
    if (_robomas_phcan_global != phcan)return;
    _RoboMas_PopSendTx8Bytes(phcan, &_robomas_can_buf_ring1);
}

void RoboMas_WhenTxMailboxAbortCallbackCalled(CAN_HandleTypeDef *phcan) {
    if (_robomas_phcan_global != phcan)return;
    _RoboMas_PopSendTx8Bytes(phcan, &_robomas_can_buf_ring1);
}


void _set_fb_data_raw(const uint8_t rxData[], uint8_t device_id) {
    if (device_id > 9 || device_id <= 0)return;
    robomas_feedback_data_raw* fb_data_row = &_robomas_feedback_data_raw_global[device_id];

    fb_data_row->_get_counter += 1;
    if (fb_data_row->_get_counter > 128) {
        fb_data_row->_get_counter = 128;  // overflow対策
    }

    if (fb_data_row->_get_counter < ROBOMAS_FEEDBACK_READY_COUNT) {  // Encoderの初期位置を取得
        fb_data_row->_internal_offset_pos = (uint16_t) (rxData[0] << 8 | rxData[1]);
        fb_data_row->pos_pre = (uint16_t) (rxData[0] << 8 | rxData[1]);
        fb_data_row->pos = (uint16_t) (rxData[0] << 8 | rxData[1]);
        return;
    }

    // dataの設定
    fb_data_row->pos_pre = fb_data_row->pos;
    fb_data_row->pos = (uint16_t) (rxData[0] << 8 | rxData[1]);
    fb_data_row->vel = (int16_t) (rxData[2] << 8 | rxData[3]);
    fb_data_row->cur = (int16_t) (rxData[4] << 8 | rxData[5]);

    // 回転数の計算
    int32_t diff_pos = (int32_t) (fb_data_row->pos) - (int32_t) (fb_data_row->pos_pre);
    if (diff_pos > 4096) {
        if (fb_data_row->_rot_num != -(INT64_MAX / 10)) {
            fb_data_row->_rot_num -= 1;
        }  // overflow対策
    } else if (diff_pos < -4096) {
        if (fb_data_row->_rot_num != (INT64_MAX / 10)) {
            fb_data_row->_rot_num += 1;
        }  // overflow対策
    }
}


void RoboMas_WhenCANRxFifo0MsgPending(CAN_HandleTypeDef *phcan) {
    // 各CANで独立したFIFO0&FIFO1を持っている
    if (_robomas_phcan_global != phcan)return;
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    if (HAL_CAN_GetRxMessage(phcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
        // Reception Error
        printf("GetRxMessage error\n\r");
        Error_Handler();
    }

    if (((rxHeader.StdId - 0x200) < 9) && ((rxHeader.StdId - 0x200) >= 1)) {
        _set_fb_data_raw(rxData, rxHeader.StdId - 0x200);  // fb_data_rawにデータを入力
    }
}


void RoboMas_WhenCANRxFifo1MsgPending(CAN_HandleTypeDef *phcan) {
    // 各CANで独立したFIFO0&FIFO1を持っている
    if (_robomas_phcan_global != phcan)return;
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    if (HAL_CAN_GetRxMessage(phcan, CAN_RX_FIFO1, &rxHeader, rxData) != HAL_OK) {
        // Reception Error
        printf("GetRxMessage error\n\r");
        Error_Handler();
    }

    if (((rxHeader.StdId - 0x200) < 9) && ((rxHeader.StdId - 0x200) >= 1)) {
        _set_fb_data_raw(rxData, rxHeader.StdId - 0x200);  // fb_data_rawにデータを入力
    }
}


void Init_RoboMas_CAN_System(CAN_HandleTypeDef *phcan) {  //CAN初期化
    _robomas_phcan_global = phcan;

    /* 受信通知を有効化する前に、接続判定用の状態を初期化する。 */
    for (uint8_t i = 0; i < 9; i++) {  // init fb_data_raw
        _robomas_feedback_data_raw_global[i].pos = 0;
        _robomas_feedback_data_raw_global[i].pos_pre = 0;
        _robomas_feedback_data_raw_global[i]._rot_num = 0;
        _robomas_feedback_data_raw_global[i].vel = 0;
        _robomas_feedback_data_raw_global[i].cur = 0;
        _robomas_feedback_data_raw_global[i]._get_counter = 0;
        _robomas_feedback_data_raw_global[i]._internal_offset_pos = 0;
    }

    CAN_FilterTypeDef sFilterConfig;

    //フィルタバンク設定
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    // CAN2をつかうならSlaveStartFilterBank以上の値をFilterBankに設定する必要がある (27まで)
    // CAN3を使う場合はFilterBankは13までっぽい (stm32f7xx_hal_can.c参照)
    if (phcan->Instance == CAN1 || phcan->Instance == CAN3){
        // CAN1 or 3 使用時のフィルタを設定 (0x201 ~ 0x208)

        // ID 2, 6 からの FB のフィルタを FIFO 0 に設定
        sFilterConfig.FilterBank = 2;
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
        sFilterConfig.FilterIdHigh = (0x200 | 0b0010) << 5;
        sFilterConfig.FilterMaskIdHigh = (((1 << 12) - 1) ^ 0b0100) << 5;
        sFilterConfig.FilterIdLow = 0b000; // 下16bit
        sFilterConfig.FilterMaskIdLow = (1 << 16) - 1;  // Standard ID
        if (HAL_CAN_ConfigFilter(phcan, &sFilterConfig) != HAL_OK) {
            Error_Handler();
        }

        // ID 4 からの FB のフィルタを FIFO 0 に設定
        sFilterConfig.FilterBank = 3;
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
        sFilterConfig.FilterIdHigh = (0x200 | 0b0100) << 5;
        sFilterConfig.FilterMaskIdHigh = ((1 << 12) - 1) << 5;
        sFilterConfig.FilterIdLow = 0b000; // 下16bit
        sFilterConfig.FilterMaskIdLow = (1 << 16) - 1;  // Standard ID
        if (HAL_CAN_ConfigFilter(phcan, &sFilterConfig) != HAL_OK) {
            Error_Handler();
        }

        // ID 8 からの FB のフィルタを FIFO 0 に設定
        sFilterConfig.FilterBank = 4;
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
        sFilterConfig.FilterIdHigh = (0x200 | 0b1000) << 5;
        sFilterConfig.FilterMaskIdHigh = ((1 << 12) - 1) << 5;
        sFilterConfig.FilterIdLow = 0b000; // 下16bit
        sFilterConfig.FilterMaskIdLow = (1 << 16) - 1;  // Standard ID
        if (HAL_CAN_ConfigFilter(phcan, &sFilterConfig) != HAL_OK) {
            Error_Handler();
        }

        // それ以外のフィルタはFIFO1に設定
        // ID 1, 3, 5, 7 からの FB のフィルタを FIFO 1 に設定
        sFilterConfig.FilterBank = 5;
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
        sFilterConfig.FilterIdHigh = (0x200 | 0b0001) << 5;
        sFilterConfig.FilterMaskIdHigh = (((1 << 12) - 1) ^ 0b0110) << 5;
        sFilterConfig.FilterIdLow = 0b000; // 下16bit
        sFilterConfig.FilterMaskIdLow = (1 << 16) - 1;  // Standard ID
        if (HAL_CAN_ConfigFilter(phcan, &sFilterConfig) != HAL_OK) {
            Error_Handler();
        }

    } else if (phcan->Instance == CAN2) {
        // CAN2 使用時のフィルタを設定 (0x201 ~ 0x208)

        // ID 2, 6 からの FB のフィルタを FIFO 0 に設定
        sFilterConfig.FilterBank = 15;
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
        sFilterConfig.FilterIdHigh = (0x200 | 0b0010) << 5;
        sFilterConfig.FilterMaskIdHigh = (((1 << 12) - 1) ^ 0b0100) << 5;
        sFilterConfig.FilterIdLow = 0b000; // 下16bit
        sFilterConfig.FilterMaskIdLow = (1 << 16) - 1;  // Standard ID
        if (HAL_CAN_ConfigFilter(phcan, &sFilterConfig) != HAL_OK) {
            Error_Handler();
        }

        // ID 4 からの FB のフィルタを FIFO 0 に設定
        sFilterConfig.FilterBank = 16;
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
        sFilterConfig.FilterIdHigh = (0x200 | 0b0100) << 5;
        sFilterConfig.FilterMaskIdHigh = ((1 << 12) - 1) << 5;
        sFilterConfig.FilterIdLow = 0b000; // 下16bit
        sFilterConfig.FilterMaskIdLow = (1 << 16) - 1;  // Standard ID
        if (HAL_CAN_ConfigFilter(phcan, &sFilterConfig) != HAL_OK) {
            Error_Handler();
        }

        // ID 8 からの FB のフィルタを FIFO 0 に設定
        sFilterConfig.FilterBank = 17;
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
        sFilterConfig.FilterIdHigh = (0x200 | 0b1000) << 5;
        sFilterConfig.FilterMaskIdHigh = ((1 << 12) - 1) << 5;
        sFilterConfig.FilterIdLow = 0b000; // 下16bit
        sFilterConfig.FilterMaskIdLow = (1 << 16) - 1;  // Standard ID
        if (HAL_CAN_ConfigFilter(phcan, &sFilterConfig) != HAL_OK) {
            Error_Handler();
        }

        // それ以外のフィルタはFIFO1に設定
        // ID 1, 3, 5, 7 からの FB のフィルタを FIFO 1 に設定
        sFilterConfig.FilterBank = 18;
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
        sFilterConfig.FilterIdHigh = (0x200 | 0b0001) << 5;
        sFilterConfig.FilterMaskIdHigh = (((1 << 12) - 1) ^ 0b0110) << 5;
        sFilterConfig.FilterIdLow = 0b000; // 下16bit
        sFilterConfig.FilterMaskIdLow = (1 << 16) - 1;  // Standard ID
        if (HAL_CAN_ConfigFilter(phcan, &sFilterConfig) != HAL_OK) {
            Error_Handler();
        }

    } else {
        printf("CAN Instance Error\n");
        Error_Handler();
    }


    if (HAL_CAN_Start(phcan) != HAL_OK) {
        printf(" -> Start Error CAN_C620\n");
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(phcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK){
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

}


RoboMas_FeedbackData Get_RoboMas_FeedbackData(RoboMas_DeviceInfo *device_info) {
    uint8_t device_id = device_info->device_id;
    if (device_id >= 9)device_id = 0;

    RoboMas_FeedbackData fb_data;
    fb_data.device_id = device_id;
    robomas_feedback_data_raw *fb_data_row = &(_robomas_feedback_data_raw_global[device_id]);

    int32_t offset_pos = (int32_t) (fb_data_row->pos) - (int32_t) (fb_data_row->_internal_offset_pos);
    if (device_info->ctrl_param.use_internal_offset != ROBOMAS_USE_OFFSET_POS_DISABLE) {
        fb_data.position = ((float)offset_pos) / 8192.0f + (float) (fb_data_row->_rot_num);
    } else {
        fb_data.position = ((float)fb_data_row->pos) / 8192.0f + (float) (fb_data_row->_rot_num);
    }

    fb_data.velocity = ((float) (fb_data_row->vel)) / 60.0f;
    switch (device_info->device_type) {
    	case ROBOMASTER_C610:
    		fb_data.current = ((float) (fb_data_row->cur * 10)) / 10000.0f;
    		break;
    	case ROBOMASTER_C620:
    	    fb_data.current = ((float) (fb_data_row->cur * 20)) / 16384.0f;
    	    break;
    	default:
    	    fb_data.current = 0.0f;
    	    break;
    }
    fb_data.get_flag = (fb_data_row->_get_counter >= ROBOMAS_FEEDBACK_READY_COUNT);

    fb_data.velocity *= device_info->ctrl_param.quant_per_rot;
    fb_data.position *= device_info->ctrl_param.quant_per_rot;
    if(device_info->ctrl_param.rotation == ROBOMAS_ROT_CW){  // TODO: 反転処理(確認)
        fb_data.velocity *= -1.0f;
        fb_data.position *= -1.0f;
    }
    fb_data.position += device_info->ctrl_param.offset_pos;
    return fb_data;
}

void _change_internal_offset_for_calib(RoboMas_DeviceInfo *device_info){
    if (device_info->device_id > 9 || device_info->device_id <= 0)return;
    robomas_feedback_data_raw* fb_data_row = &_robomas_feedback_data_raw_global[device_info->device_id];

    fb_data_row->_internal_offset_pos = fb_data_row->pos;  // 現在の位置をoffsetに設定
    fb_data_row->_rot_num = 0;  // 回転数をリセット
}

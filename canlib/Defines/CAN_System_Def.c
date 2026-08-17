//
// Created by emile on 2021/10/13.
//

#include "CAN_System_Def.h"
#include "main.h"


HAL_StatusTypeDef _CANLib_PushTx8Bytes(CANRingBuf_ExtID *const p_can_buffer, const uint32_t ExtId, const uint8_t *const bytes, const uint32_t size){
    p_can_buffer->buffer[p_can_buffer->write_point].DLC = size;
    p_can_buffer->buffer[p_can_buffer->write_point].ExtId = ExtId;
    for (uint8_t i = 0; i < size; i++)p_can_buffer->buffer[p_can_buffer->write_point].bytes[i] = bytes[i];

    if (p_can_buffer->is_full == 1) p_can_buffer->read_point = (p_can_buffer->read_point + 1) & (CANLIB_CAN_TXBUFFER_SIZE - 1);
    p_can_buffer->write_point = (p_can_buffer->write_point + 1) & (CANLIB_CAN_TXBUFFER_SIZE - 1);

    if (p_can_buffer->write_point == p_can_buffer->read_point){
        p_can_buffer->is_full = 1;
    }
    return HAL_OK;
}

HAL_StatusTypeDef _CANLib_PopSendTx8Bytes(CANRingBuf_ExtID *const p_can_buffer, CAN_HandleTypeDef *const phcan){
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;

    txHeader.RTR = CAN_RTR_DATA; // Data frame
    txHeader.IDE = CAN_ID_EXT;     // CAN Extend ID
    txHeader.TransmitGlobalTime = DISABLE;

    while (HAL_CAN_GetTxMailboxesFreeLevel(phcan) > 0){
        if (p_can_buffer->is_full == 0 && p_can_buffer->read_point == p_can_buffer->write_point)break;

        txHeader.DLC = p_can_buffer->buffer[p_can_buffer->read_point].DLC;
        txHeader.ExtId = p_can_buffer->buffer[p_can_buffer->read_point].ExtId;

        HAL_StatusTypeDef ret = HAL_CAN_AddTxMessage(
            phcan, &txHeader, p_can_buffer->buffer[p_can_buffer->read_point].bytes, &txMailbox);
        if (ret != HAL_OK)return ret;

        p_can_buffer->read_point = (p_can_buffer->read_point + 1) & (CANLIB_CAN_TXBUFFER_SIZE - 1);
        p_can_buffer->is_full = 0;
    }
    return HAL_OK;
}

HAL_StatusTypeDef _CANLib_SendBytes(CAN_HandleTypeDef *const phcan, CANRingBuf_ExtID *const p_can_buffer, const uint32_t ExtId, const uint8_t *const bytes, const uint32_t size) {
    // CANのmessageを送信する関数
    uint32_t quotient = size / 8;
    uint32_t remainder = size - (8 * quotient);
    HAL_StatusTypeDef ret;

    for (uint8_t i = 0; i < quotient; i++){
        ret = _CANLib_PushTx8Bytes(p_can_buffer, ExtId, bytes + i * 8, 8);
        if (ret != HAL_OK){
            Error_Handler();
            return ret;
        }
    }

    if(remainder != 0){
        ret = _CANLib_PushTx8Bytes(p_can_buffer, ExtId, bytes + quotient * 8, remainder);
        if (ret != HAL_OK){
            Error_Handler();
            return ret;
        }
    }

    ret = _CANLib_PopSendTx8Bytes(p_can_buffer, phcan);
    if (ret != HAL_OK){
        Error_Handler();
        return ret;
    }
    return HAL_OK;
}


/**
 * @brief CANのIDを設定する. 上16bitのみを使用
 * @param _can_device
 * @param cmd
 */
uint64_t Make_CAN_ID_from_CAN_Device(const CAN_Device *const _can_device, const uint8_t cmd){  // mainからmcmdなどへの送信
    uint8_t node_type = (uint8_t)(_can_device->node_type) & (0b111);
    return (((node_type&0b111)<<11) | (((_can_device->node_id)&0b111)<<8) | (((_can_device->device_num)&0b111)<<5) | (cmd&0b11111) );
}


/**
 * @brief CANのIDを設定する. 上16bitのみを使用
 * @param node_type
 * @param node_id
 * @param device_num
 * @param cmd
 */
uint64_t Make_CAN_ID(const Node_Type node_type, const uint8_t node_id, const uint8_t device_num, const uint8_t cmd){
    return ( (((uint8_t)node_type & 0b111) << 11) | (((node_id)&0b111) << 8) | ((device_num&0b111) << 5) | (cmd&0b11111) );
}

CAN_Device Extract_CAN_Device(const uint64_t can_id) {  // CAN_IDからCAN_Deviceを抽出する
    CAN_Device ans;
    ans.device_num = ((can_id>>5) & 0b111);
    ans.node_id = ((can_id>>8) & 0b111);
    ans.node_type = (Node_Type)( (can_id>>11) & 0b111);
    return ans;
}

uint8_t Extract_CAN_CMD(const uint64_t can_id){ return ( can_id & 0b11111); }  // cmdを抽出

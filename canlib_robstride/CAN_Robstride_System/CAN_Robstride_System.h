/*
 * CAN_Main_RoboMas.h
 *
 *  Created on: 7 8, 2023
 *      Author: Emile
 */

#ifndef CAN_ROBSTRIDE_SYSTEM_H_
#define CAN_ROBSTRIDE_SYSTEM_H_

// Includes --------------------------------

#include "main.h"
#include "CAN_Robstride_Def.h"

// Defines --------------------------------

#define CAN_TXBUFFER_SIZE (512)

// Typedefs --------------------------------

typedef struct {
    uint32_t ExtId; // 18bit
    uint32_t DLC;
    uint8_t bytes[8];
} CANTxBuf_Robstride;

typedef struct {
    CANTxBuf_Robstride buffer[CAN_TXBUFFER_SIZE];
    uint32_t read_point;
    uint32_t write_point;
    uint8_t is_full;
} CAN_RingBuf_Robstride;

// Function Prototypes --------------------------------

// Mailbox2を使う
void Robstride_WhenTxMailboxCompleteCallbackCalled(CAN_HandleTypeDef *phcan);

void Robstride_WhenTxMailboxAbortCallbackCalled(CAN_HandleTypeDef *phcan);

void Robstride_WhenCANRxFifo0MsgPending(CAN_HandleTypeDef *phcan);

void Robstride_WhenCANRxFifo1MsgPending(CAN_HandleTypeDef *phcan);

void Get_Robstride_MCUID(const uint8_t rxData[], uint8_t device_id);

void Robstride_SetCANID(Robstride_DeviceInfo *device_info, uint8_t new_id);

void Robstride_ProcessParameter(const uint8_t rxData[], uint8_t device_id);

void Robstride_ProcessFault(const uint8_t rxData[], uint8_t device_id);

void Robstride_RequestReadParameter(Robstride_DeviceInfo *device_info, uint16_t address);

void Robstride_WriteFloatData(Robstride_DeviceInfo *device_info, uint16_t address, float data);

void Robstride_WriteIntData(Robstride_DeviceInfo *device_info, uint16_t address, int data);

HAL_StatusTypeDef Robstride_SendBytes(CAN_HandleTypeDef *phcan, uint8_t motor_id, uint8_t cmd_id, uint16_t option, const uint8_t *bytes, uint32_t size);

void Init_Robstride_CAN_System(CAN_HandleTypeDef *phcan);

void Robstride_fb_init(Robstride_DeviceInfo *device_info);

Robstride_FeedbackData Get_Robstride_FeedbackData(Robstride_DeviceInfo *device_info);  // Robstride からデータを読みだして FB を取得
Robstride_FeedbackData Read_Robstride_FeedbackData(Robstride_DeviceInfo *device_info); // 以前取得して保管されている Robstride の FB データを返す（新たに通信を行わない）

#endif

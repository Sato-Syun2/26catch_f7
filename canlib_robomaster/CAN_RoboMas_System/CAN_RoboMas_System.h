/*
 * CAN_Main_RoboMas.h
 *
 *  Created on: 7 8, 2023
 *      Author: Emile
 */

#ifndef CAN_ROBOMAS_SYSTEM_H_
#define CAN_ROBOMAS_SYSTEM_H_

#include "main.h"
#include "CAN_RoboMas_Def.h"


// Mailbox2を使う
void RoboMas_WhenTxMailboxCompleteCallbackCalled(CAN_HandleTypeDef *phcan);

void RoboMas_WhenTxMailboxAbortCallbackCalled(CAN_HandleTypeDef *phcan);

void RoboMas_WhenCANRxFifo0MsgPending(CAN_HandleTypeDef *phcan);

void RoboMas_WhenCANRxFifo1MsgPending(CAN_HandleTypeDef *phcan);

HAL_StatusTypeDef RoboMas_SendBytes(CAN_HandleTypeDef *phcan, uint32_t StdId, uint8_t *bytes, uint32_t size);

void Init_RoboMas_CAN_System(CAN_HandleTypeDef *phcan);

RoboMas_FeedbackData Get_RoboMas_FeedbackData(RoboMas_DeviceInfo *device_info);

void _change_internal_offset_for_calib(RoboMas_DeviceInfo *device_info);  // don't use.


#endif

/*
 * CAN_AirCylinder.h
 *
 * Created on: 2/9/2022
 *     Author: Kotaro Okuda
 */

#ifndef _CAN_AIRCYLINDER_H
#define _CAN_AIRCYLINDER_H

#include "CAN_System_Def.h"
#include "CAN_AirCylinder_Def.h"
#include "aircylinder.h"

#define CAN_RXBUFFER_SIZE (512)

extern Air_HandleTypedef air_global[8];

/* @brief: This function is called with Timer Interruption
 */
HAL_StatusTypeDef Process_Fifo0Msg(void);

/* Fifo0MsgPendingで呼び出すこと
 * Fifo0MsgPendingはCANメッセージをFifo0に受信したときに呼び出される
 * コールバック関数である.
 */
void WhenCANRxFifo0MsgPending();

/* Fifo1MsgPendingで呼び出すこと
 * ただし, stm32f3xx系はDual_CANをサポートしていないらしいので,
 * Fifo1はそもそも使用しないが, 形式的に記述している.
 */
void WhenCANRxFifo1MsgPending();

/* @brief: Initialize AirCylinder with CAN
 */
void InitCANAir(CAN_HandleTypeDef *hcan);

/* @brief: Get circuit's ID from Rotary Switch
 */
uint8_t GetID();

/* @brief: Print Aircylinder Parameters
 */ 
void Print_AirPortStatus(Air_HandleTypedef *air_global);

/* @brief: AirCylinder Port Status Update with Timer Interruption
 */
void CANAirUpdate();

#endif /* _CAN_AIRCYLINDER_H */

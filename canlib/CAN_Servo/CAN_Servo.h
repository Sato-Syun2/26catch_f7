//
// Created by emile on 2021/11/04.
//

#ifndef _CAN_SERVO_H
#define _CAN_SERVO_H

#include "CAN_System_Def.h"
#include "CAN_Servo_Def.h"
#include "servo.h"

#define CAN_RXBUFFER_SIZE (512)

extern Servo_HandleTypedef servo_global[4];

HAL_StatusTypeDef Process_Fifo0Msg(void);
void WhenCANRxFifo0MsgPending();	//Fifo0MsgPendingで呼び出すこと
void WhenCANRxFifo1MsgPending();	//Fifo1MsgPendingで呼び出すこと


void InitCANServo(CAN_HandleTypeDef* hcan);
uint8_t GetID();
void PrintServoParams(Servo_HandleTypedef* hmcan);
void CANServoUpdate();

#endif //_CAN_SERVO_H

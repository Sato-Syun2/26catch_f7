/*
 * CAN_AirCylinder_Def.h
 *
 * Created on: 2/9/2022
 *     Author: Kotaro Okuda
 */

#ifndef _CAN_AIRCYLINDER_DEF_H_
#define _CAN_AIRCYLINDER_DEF_H_


typedef enum {
    AIR_OFF = 0U, // Equal to GPIO_PIN_RESET
    AIR_ON        // Equal to GPIO_PIN_SET
} Air_PortStatus_Typedef;

typedef enum {
    PORT_1 = 0,
    PORT_2 = 1,
    PORT_3 = 2,
    PORT_4 = 3,
    PORT_5 = 4,
    PORT_6 = 5,
    PORT_7 = 6,
    PORT_8 = 7,
} Air_PortNumber_Typedef;


// エアシリンダのCAN送受信用コマンド
typedef enum CANAir_Main_CMD{
    AIR_CMD_AWAKE,
    AIR_CMD_INIT,
    AIR_CMD_OUTPUT,
}CANAir_CMD;

#endif /* _CAN_AIRCYLINDER_DEF_H_ */

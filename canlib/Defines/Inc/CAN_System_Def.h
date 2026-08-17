/*
 * CAN_System_Def.h
 *
 *  Created on: 9/25, 2021
 *      Author: Emile
 */

#ifndef INC_CAN_SYSTEM_CAN_SYSTEM_DEF_H_
#define INC_CAN_SYSTEM_CAN_SYSTEM_DEF_H_


#include <main.h>

#define CANLIB_CAN_TXBUFFER_SIZE (512)


typedef enum{
    NODE_MAIN = 0,  // メイン基板
    NODE_DUMMY_1, // 今後の追加のために空けておく
    NODE_VL53L0X, // VL53L0X 基板
    NODE_DUMMY_3, // 今後の追加のために空けておく
    NODE_SERVO,     // servoの基板
    NODE_AIR,       // エアシリンダ
    NODE_MCMD4,     // device 0,1 100A モタドラ
    NODE_DYNAMIXEL_KONDO, // DYNAMIXEL_KONDO 基板
    NODE_TYPES      //nodeの種類数:必ず列挙体の最後にする
}Node_Type;


typedef struct{
    Node_Type node_type;  // 基板の種類
    uint8_t node_id;      // 基板の番号
    uint8_t device_num;   // 動かすデバイス(motorとか)の番号
}CAN_Device;


typedef struct{
    uint32_t ExtId; // 18bit
    uint32_t DLC;
    uint8_t bytes[8];
} CANTxBuf_ExtID;


typedef struct {
    CANTxBuf_ExtID buffer[CANLIB_CAN_TXBUFFER_SIZE];
    uint32_t read_point;
    uint32_t write_point;
    uint8_t is_full;
} CANRingBuf_ExtID;


// main.cなどからは叩かない関数
HAL_StatusTypeDef _CANLib_PushTx8Bytes(CANRingBuf_ExtID *p_can_buffer, uint32_t ExtId, const uint8_t *bytes, uint32_t size);
HAL_StatusTypeDef _CANLib_PopSendTx8Bytes(CANRingBuf_ExtID *p_can_buffer, CAN_HandleTypeDef *phcan);
HAL_StatusTypeDef _CANLib_SendBytes(CAN_HandleTypeDef *phcan, CANRingBuf_ExtID *p_can_buffer, uint32_t ExtId, const uint8_t *bytes, uint32_t size);

uint64_t Make_CAN_ID_from_CAN_Device(const CAN_Device *_can_device, uint8_t cmd);  // CAN_IDを生成する
uint64_t Make_CAN_ID(Node_Type node_type, uint8_t node_id, uint8_t device_num, uint8_t cmd);  // CAN_IDを生成する
CAN_Device Extract_CAN_Device(uint64_t can_id);  // CAN_IDからCAN_Deviceを抽出する
uint8_t Extract_CAN_CMD(uint64_t can_id);  // CAN_IDからcmdを抽出する

/*
 *  Base ID : 14bit
 *  Data fieldのData : Max 64bit
 *
 * メインからモタドラなどへの送信
 *---------------------------
 * BIT     |内容
 * --------|-----------------
 * [13:11] |Destination NodeType
 * --------|-----------------
 * [10:8]  |Destination NodeID
 * --------|-----------------
 * [7:5]   | DeviceNum
 * --------|-----------------
 * [4:0]   | CommandType
 *---------------------------
 *
 * Destinationのところだけを取ってくるなら,
 * MaskIdHigh = MAKE_MAIN_CANID(0b111, 0b111, 0) << 5; とすればok.
 * 16bitの前14bitがIDに対応する. この14bitをいいかんじに操作してやる.
 *モタドラなどからメインへの送信
 *---------------------------
 * BIT     |内容
 * --------|-----------------
 * [13:11] |Source NodeType
 * --------|-----------------
 * [10:8]  |Source NodeID
 * --------|-----------------
 * [7:5]   | DeviceNum
 * --------|-----------------
 * [4:0]   | CommandType
 *---------------------------
 *
 * フローを書く
 */


#define CMD_AWAKE 0x0

#endif /* INC_CAN_SYSTEM_CAN_SYSTEM_DEF_H_ */

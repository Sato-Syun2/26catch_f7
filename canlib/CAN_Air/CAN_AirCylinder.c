/*
 * CAN_AirCylinder.c
 *
 * Created on: 2/9/2022
 *     Author: Kotaro Okuda
 */

#include "CAN_AirCylinder.h"
#include "string.h"

CAN_HandleTypeDef* phcan;
uint8_t node_id;

Air_HandleTypedef air_global[8];


HAL_StatusTypeDef SendBytes(CAN_HandleTypeDef* hcan, uint32_t ExtId, uint8_t* bytes, uint32_t size) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;

    TxHeader.ExtId = ExtId;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_EXT;
    TxHeader.DLC = size;
    TxHeader.TransmitGlobalTime = DISABLE;

    uint32_t quotient = size / 8;
    uint32_t remainder = size - (8 * quotient);

    HAL_StatusTypeDef ret;

    for (uint8_t i = 0; i < quotient; i++) {
        //以下のDelayは経験的につけている(2/9/2022現在)
        HAL_Delay(2);
        ret = HAL_CAN_AddTxMessage(hcan, &TxHeader, bytes + i * 8, &TxMailbox);
        if (ret != HAL_OK) {
            Error_Handler();
            return ret; //おそらく無意味
        }
    }

    TxHeader.DLC = remainder;
    if (TxHeader.DLC != 0) {
        //以下のDelayは経験的につけている(2/9/2022現在)
        HAL_Delay(2);
        ret = HAL_CAN_AddTxMessage(hcan, &TxHeader, bytes + quotient * 8, &TxMailbox);
        if (ret != HAL_OK) {
            Error_Handler();
            return ret;
        }
    }

    return HAL_OK;
}

// バッファ用構造体の定義
typedef struct {
    uint8_t port_num;
    uint8_t cmd;
    uint8_t rxData[8];
} CANRxBuf;

// バッファの用意
CANRxBuf buffer[CAN_RXBUFFER_SIZE];
uint32_t readpoint = 0;
uint32_t writepoint = 0;
uint8_t isfull = 0;

HAL_StatusTypeDef PushTx8Bytes(uint8_t port_num, uint8_t cmd, uint8_t* rxData) {
    //	printf("PushTx8bytes\n\r");
    buffer[writepoint].port_num = port_num;
    buffer[writepoint].cmd = cmd;
    for (uint8_t i = 0; i < 8; i++)
        buffer[writepoint].rxData[i] = rxData[i];

    if (isfull == 1)
        readpoint = (readpoint + 1) & (CAN_RXBUFFER_SIZE - 1);
    writepoint = (writepoint + 1) & (CAN_RXBUFFER_SIZE - 1);

    if (writepoint == readpoint) {
        isfull = 1;
    }

    return HAL_OK;
}

HAL_StatusTypeDef Process_Fifo0Msg(void) {
    //	printf("Process_Fifo0Msg\n\r");
    Air_PortStatus_Typedef port_status;
    while (isfull != 0 || readpoint != writepoint) {
        uint8_t port_num = buffer[readpoint].port_num;
        switch (buffer[readpoint].cmd) {
            case AIR_CMD_INIT:
                //            printf("AIR_CMD_INIT\n\r");
                memcpy(&port_status, buffer[readpoint].rxData, sizeof(port_status));
                Air_SetPortStatus(&air_global[port_num], port_status);
                air_global[port_num]._is_initialized = 1;
                //            printf("port_num = %d\n\r", port_num);
                break;
            case AIR_CMD_OUTPUT:
                //            printf("AIR_CMD_OUTPUT\n\r");
                memcpy(&port_status, buffer[readpoint].rxData, sizeof(port_status));
                Air_SetPortStatus(&air_global[port_num], port_status);
                //            printf("port_num = %d\n\r", port_num);
                break;
            default:
                //        	printf("default\n\r");
                break;
        }
        // Pop
        readpoint = (readpoint + 1) & (CAN_RXBUFFER_SIZE - 1);
        isfull = 0;
    }

    return HAL_OK;
}

/* @brief: AirCylinderはFifo0にデータ受信
 */
void WhenCANRxFifo0MsgPending() {
    //	printf("WCANRxFifo0MP\n\r");
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    // Get Rx Message
    if (HAL_CAN_GetRxMessage(phcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
        // Reception Error
        printf("GetRxMessage Error\n\r");
        Error_Handler();
    }
    CAN_Device can_device = Extract_CAN_Device(rxHeader.ExtId);
    uint8_t extracted_cmd = Extract_CAN_CMD(rxHeader.ExtId);
    if ((can_device.node_type != NODE_AIR) || (can_device.node_id != node_id)) {
        //他のデバイスへの命令を受信
        //        printf("Warning: Received msg to another device\n\r");
        return;
    }
    uint8_t port_num = can_device.device_num;
    PushTx8Bytes(port_num, extracted_cmd, rxData);
}

/* @brief: AirCylinderには, Fifo1は使用する予定はない.
 */
void WhenCANRxFifo1MsgPending() {
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    //Get Rx Message
    if (HAL_CAN_GetRxMessage(phcan, CAN_RX_FIFO1, &rxHeader, rxData) != HAL_OK) {
        //Reception Error
        printf("GetRxMessage Error\n\r");
        Error_Handler();
    }
    CAN_Device can_device = Extract_CAN_Device(rxHeader.ExtId);
    uint8_t extracted_cmd = Extract_CAN_CMD(rxHeader.ExtId);
    if ((can_device.node_type != NODE_AIR) || (can_device.node_id != node_id)) {
        //他のデバイスへの命令を受信
        printf("Warning: Received msg to another device\n\r");
        return;
    }
    uint8_t device_id = can_device.device_num;
    PushTx8Bytes(device_id, extracted_cmd, rxData);
}

void InitCANAir(CAN_HandleTypeDef* hcan) {
    phcan = hcan;
    node_id = GetID(); // node_idを取得する(ロータリースイッチで基板番号の取得)
    printf("%d\n\r", node_id);
    for (uint8_t i = 0; i < 8; i++) {
        air_global[i]._is_initialized = 0; //構造体の初期化
    }
    // CAN初期化
    CAN_FilterTypeDef sFilterConfig;
    printf("InitCANAir start\n\r");

    //ここからCANフィルターの設定
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; // IDMASKモード
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 32bitモード
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; //FIFO割り当て
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14; //慣習的にいれているが, stm32f3xx系の場合この設定に意味はないと思われる.

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_AIR, node_id, 0, AIR_CMD_INIT) >> 13;
    sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0b111, 0b111, 0, 0b11111) >> 13;
    sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_AIR, node_id, 0, AIR_CMD_INIT) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0b111, 0b111, 0, 0b11111) & 0x1FFF) << 3 | 0b100;

    //フィルターを登録
    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) {
        /* Filter Configuration Error */
        Error_Handler();
    }

    sFilterConfig.FilterBank = 1;
    sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_AIR, node_id, 0, AIR_CMD_OUTPUT) >> 13;
    sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0b111, 0b111, 0, 0b11111) >> 13;
    sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_AIR, node_id, 0, AIR_CMD_OUTPUT) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0b111, 0b111, 0, 0b11111) & 0x1FFF) << 3 | 0b100;

    //フィルターを登録
    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) {
        /* Filter Configuration Error */
        Error_Handler();
    }

    printf("CAN Start \n\r");
    if (HAL_CAN_Start(hcan) != HAL_OK) {
        /* Start Error */
        printf("Start Error\n\r");
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        printf("FIFO0 CAN_Activation error\n\r");
        Error_Handler();
    }

    //形式的にFifo1についても記述している.
    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) {
        printf("FIFO1 CAN_Activation error\n\r");
        Error_Handler();
    }


    printf("CAN Interrupt Activated \n\r");

    while ((!air_global[0]._is_initialized) || (!air_global[1]._is_initialized) || (!air_global[2]._is_initialized) || (
               !air_global[3]._is_initialized) ||
           (!air_global[4]._is_initialized) || (!air_global[5]._is_initialized) || (!air_global[6]._is_initialized) || (
               !air_global[7]._is_initialized)) {
        // すべてのAir_Portが初期化されるまで待機
        if (HAL_CAN_GetError(hcan) == HAL_CAN_ERROR_BOF) {
            printf("error\n\r");
            HAL_CAN_Init(hcan);
        }
        SendBytes(hcan, Make_CAN_ID(NODE_AIR, node_id, 0, AIR_CMD_AWAKE), &node_id, sizeof(uint8_t));
        Process_Fifo0Msg();
        printf("send awake cmd\n\r");
        HAL_Delay(1000);
        printf("%d\n\r", air_global[0]._is_initialized);
        printf("%d\n\r", air_global[1]._is_initialized);
        printf("%d\n\r", air_global[2]._is_initialized);
        printf("%d\n\r", air_global[3]._is_initialized);
        printf("%d\n\r", air_global[4]._is_initialized);
        printf("%d\n\r", air_global[5]._is_initialized);
        printf("%d\n\r", air_global[6]._is_initialized);
        printf("%d\n\r", air_global[7]._is_initialized);
    }
}

uint8_t GetID() {
    return (HAL_GPIO_ReadPin(ID1_GPIO_Port, ID1_Pin)) | (HAL_GPIO_ReadPin(ID2_GPIO_Port, ID2_Pin) << 1) | (
               HAL_GPIO_ReadPin(ID4_GPIO_Port, ID4_Pin) << 2);
}

void Print_AirPortStatus(Air_HandleTypedef* air) {
    printf("PORT %d: status %d; \n\r", air->port_num, air->port_status);
}

void CANAirUpdate() {
    for (uint8_t i = 0; i < 8; ++i) {
        if (air_global[i]._is_initialized)
            Air_Update(&air_global[i]);
    }
}

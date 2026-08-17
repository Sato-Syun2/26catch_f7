//
// Created by emile on 2021/11/04.
//

#include "CAN_Servo.h"
#include "string.h"
CAN_HandleTypeDef *phcan;
uint8_t node_id;
uint8_t is_initialized=0;

Servo_HandleTypedef servo_global[4];


HAL_StatusTypeDef SendBytes(CAN_HandleTypeDef *hcan, uint32_t ExtId, uint8_t* bytes, uint32_t size){
    CAN_TxHeaderTypeDef   TxHeader;
    uint32_t TxMailbox;

    TxHeader.ExtId = ExtId;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_EXT;
    TxHeader.DLC = size;
    TxHeader.TransmitGlobalTime = DISABLE;

    uint32_t quotient = size / 8;
    uint32_t remainder = size - (8 * quotient);

    HAL_StatusTypeDef ret;

    for(uint8_t i = 0; i < quotient; i++){
        HAL_Delay(2);//FIFOの空きを見るべき
        ret = HAL_CAN_AddTxMessage(hcan, &TxHeader, bytes + i*8, &TxMailbox);
        if(ret != HAL_OK){
            Error_Handler();
            return ret;
        }else{

        }
    }

    TxHeader.DLC = remainder;
    if(TxHeader.DLC!=0){
        HAL_Delay(2);//FIFOの空きを見るべき
        ret = HAL_CAN_AddTxMessage(hcan, &TxHeader, bytes + quotient*8, &TxMailbox);
        if(ret != HAL_OK){
            Error_Handler();
            return ret;
        }else{

        }
    }
    return HAL_OK;
}

typedef struct{
    uint8_t device_id;
    uint8_t cmd;
    uint8_t rxData[8];
}CANRxBuf;

CANRxBuf buffer[CAN_RXBUFFER_SIZE];
uint32_t readpoint = 0;
uint32_t writepoint = 0;
uint8_t isfull = 0;


HAL_StatusTypeDef PushTx8Bytes(uint8_t device_id, uint8_t cmd, uint8_t* rxData){
    buffer[writepoint].device_id = device_id;
    buffer[writepoint].cmd = cmd;
    for(uint8_t i = 0; i < 8; i++)buffer[writepoint].rxData[i] = rxData[i];

    if(isfull == 1)readpoint = (readpoint + 1) & (CAN_RXBUFFER_SIZE - 1);
    writepoint = (writepoint + 1) & (CAN_RXBUFFER_SIZE - 1);

    if(writepoint == readpoint){
        isfull = 1;
    }

    return HAL_OK;
}


HAL_StatusTypeDef Process_Fifo0Msg(void){
    while(isfull != 0 || readpoint != writepoint){
        float fdata[2];
        uint8_t device_id = buffer[readpoint].device_id;
        switch(buffer[readpoint].cmd){
            case SERVO_CMD_INIT1:
                if(servo_global[device_id]._is_initialized)return HAL_ERROR;
                memcpy(&fdata, buffer[readpoint].rxData, sizeof(fdata));
                servo_global[device_id].pulse_width_min = fdata[0];
                servo_global[device_id].pulse_width_max = fdata[1];
                break;
            case SERVO_CMD_INIT2:
                if(servo_global[device_id]._is_initialized)return HAL_ERROR;
                memcpy(&fdata, buffer[readpoint].rxData, sizeof(fdata));
                servo_global[device_id].angle_range = fdata[1];
                break;
            case SERVO_CMD_INIT3:
                if(servo_global[device_id]._is_initialized)return HAL_ERROR;
                memcpy(&fdata, buffer[readpoint].rxData, sizeof(fdata));
                servo_global[device_id].angle_offset = fdata[0];
                servo_global[device_id]._is_initialized = 1;
                is_initialized = 1;
                Servo_Enable(&servo_global[device_id]);
                break;
            default:
                break;
        }
        //Pop
        readpoint = (readpoint + 1) & (CAN_RXBUFFER_SIZE - 1);
        isfull = 0;
    }

    return HAL_OK;;
}


void WhenCANRxFifo0MsgPending(){//Fifo0MsgPendingで呼び出すこと. CAN受信時に呼び出される
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    // Get RX message
    if (HAL_CAN_GetRxMessage(phcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK){
        // Reception Error
        printf("GetRxMessage error\n\r");
        Error_Handler();
    }
    CAN_Device can_device = Extract_CAN_Device(rxHeader.ExtId);
    uint8_t extracted_cmd = Extract_CAN_CMD(rxHeader.ExtId);
    if((can_device.node_type != NODE_SERVO) || (can_device.node_id != node_id)){
        printf("warn get msg to another device\n\r");
        return;  // 別のdeviceへの命令を受信してる
    }
    uint8_t device_id = can_device.device_num;
    PushTx8Bytes(device_id, extracted_cmd, rxData);
}

void WhenCANRxFifo1MsgPending(){
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    // Get RX message
    if(HAL_CAN_GetRxMessage(phcan, CAN_RX_FIFO1, &rxHeader, (uint8_t*)&rxData) != HAL_OK){
        // Reception Error
        printf("GetRxMessage error\n\r");
        Error_Handler();
    }
    CAN_Device can_device = Extract_CAN_Device(rxHeader.ExtId);
    if((can_device.node_type != NODE_SERVO) || (can_device.node_id != node_id))return;
    uint8_t extracted_cmd = Extract_CAN_CMD(rxHeader.ExtId);
    if(extracted_cmd == SERVO_CMD_SET_TARGET){
        if((can_device.device_num < 4) && (can_device.device_num >= 0)){
            float value;
            memcpy(&value, rxData, sizeof(float));
            Servo_SetTarget(&servo_global[can_device.device_num], value);
        }
    }
}



void InitCANServo(CAN_HandleTypeDef* hcan){
    phcan = hcan;
    node_id = GetID();  // node_idを取得(基板番号)
    for(uint8_t i=0; i<4; i++)servo_global[i]._is_initialized = 0; // 構造体の初期化

    //CAN初期化
    CAN_FilterTypeDef  sFilterConfig;
    printf("InitCANServo start \n\r");

    // ここからCANのフィルター設定
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // IDMASKモード
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;  // 32bitモード
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_SERVO, node_id, 0, SERVO_CMD_INIT1) >> 13;
    sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0b111, 0b111, 0, 0b11111) >> 13;
    sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_SERVO, node_id, 0, SERVO_CMD_INIT1) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0b111, 0b111, 0, 0b11111) & 0x1FFF)<<3 | 0b100;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;  // Fifo0に受信

    if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)  // フィルターを登録
    {
        /* Filter configuration Error */
        Error_Handler();
    }

    sFilterConfig.FilterBank = 1;
    sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_SERVO, node_id, 0, SERVO_CMD_INIT2) >> 13;
    sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0b111, 0b111, 0, 0b11111) >> 13;
    sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_SERVO, node_id, 0, SERVO_CMD_INIT2) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0b111, 0b111, 0, 0b11111) & 0x1FFF)<<3 | 0b100;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;  // Fifo0に受信

    if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)  // フィルターを登録
    {
        /* Filter configuration Error */
        Error_Handler();
    }

    sFilterConfig.FilterBank = 2;
    sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_SERVO, node_id, 0, SERVO_CMD_INIT3) >> 13;
    sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0b111, 0b111, 0, 0b11111) >> 13;
    sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_SERVO, node_id, 0, SERVO_CMD_INIT3) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0b111, 0b111, 0, 0b11111) & 0x1FFF)<<3 | 0b100;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;  // Fifo0に受信

    if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)  // フィルターを登録
    {
        /* Filter configuration Error */
        Error_Handler();
    }

    sFilterConfig.FilterBank = 3;
    sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_SERVO, node_id, 0, SERVO_CMD_SET_TARGET) >> 13;
    sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0b111, 0b111, 0, 0b11111) >> 13;
    sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_SERVO, node_id, 0, SERVO_CMD_SET_TARGET) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0b111, 0b111, 0, 0b11111) & 0x1FFF)<<3 | 0b100;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;  // Fifo0に受信

    if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)  // フィルターを登録
    {
        /* Filter configuration Error */
        Error_Handler();
    }

    printf("CAN start \n\r");
    if (HAL_CAN_Start(hcan) != HAL_OK){
        /* Start Error */
        printf("Start Error\n");
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK){
        printf("FIFO0 CAN_Activation error\n\r");
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK){
        printf("FIFO1 CAN_Activation error\n\r");
        Error_Handler();
    }
    printf("CAN interrupt activated \n\r");

    while(!is_initialized){  // mainから初期化命令が送られて来るまで待機
        if(HAL_CAN_GetError(hcan) == HAL_CAN_ERROR_BOF){
            HAL_CAN_Init(hcan);
        }
        SendBytes(hcan, Make_CAN_ID(NODE_SERVO, node_id, 0, SERVO_CMD_AWAKE), &node_id, sizeof(uint8_t));
        Process_Fifo0Msg();
        printf("send awake cmd\n\r");
        HAL_Delay(1000);
    }
}

uint8_t GetID(){
    return (HAL_GPIO_ReadPin(ID1_GPIO_Port, ID1_Pin)) | (HAL_GPIO_ReadPin(ID2_GPIO_Port, ID2_Pin) << 1) | (HAL_GPIO_ReadPin(ID4_GPIO_Port, ID4_Pin) << 2);
}

void PrintServoParams(Servo_HandleTypedef* param){
    printf("{pulse_width_min, max=%f,%f\n\r",param->pulse_width_min,param->pulse_width_max);
    printf(" angle_range=%f\n\r",param->angle_range);
    printf(" angle_offset=%f}\n\r",param->angle_offset);
}

void CANServoUpdate(){
    for(uint8_t i=0;i<4;++i){
        if(servo_global[i]._is_initialized)Servo_Update(&servo_global[i]);
    }
}

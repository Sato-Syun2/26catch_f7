/*
 * CAN_Main.c
 *
 *  Created on: Feb 6, 2020
 *      Author: Eater
 */
#include "CAN_Main.h"
#include <stdio.h>
#include <string.h>
#include <math.h>


CAN_HandleTypeDef* _canlib_phcan;
CANRingBuf_ExtID _canlib_can_buf_ring = {0};
volatile uint8_t all_node_detected = 0;

uint8_t num_detected[NODE_TYPES] = {0};
uint8_t node_id_list[NODE_TYPES][0b111];

MCMD_Feeback_Main_typedef _feedback_table_mcmd4[10];

Dynamixel_Kondo_Feeback_Main_typedef _feedback_table_dynamixel_kondo[10];

static DYNAMIXEL_OR_KONDO dynamixel_kondo_board_initialized_type[10]={[0 ... 9] = DKBOARD_UNINITIALIZED}; // [i]: node_id が i である Dynamixel_Kondo 基板がどのタイプに初期化されたか

VL53L0X_Feeback_Main_typedef _feedback_table_vl53l0x[10];


void CANLib_WhenTxMailbox0_1_2CompleteCallbackCalled(const CAN_HandleTypeDef *const phcan){
    if(phcan != _canlib_phcan)return;
    _CANLib_PopSendTx8Bytes(&_canlib_can_buf_ring, _canlib_phcan);
}

void CANLib_WhenTxMailbox0_1_2AbortCallbackCalled(const CAN_HandleTypeDef *const phcan){
    if(phcan != _canlib_phcan)return;
    _CANLib_PopSendTx8Bytes(&_canlib_can_buf_ring, _canlib_phcan);
}

void CANLib_WhenCANRxFifo0MsgPending(CAN_HandleTypeDef *phcan, const NUM_OF_DEVICES *const num_of){ // Fifo0MsgPendingで呼び出すこと. CAN受信時に呼び出される関数
    if(phcan != _canlib_phcan)return;
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    if (HAL_CAN_GetRxMessage(_canlib_phcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK){
        // Reception Error
        printf("GetRxMessage error (FIFO0)\n\r");
        Error_Handler();
    }

    CAN_Device can_device = Extract_CAN_Device(rxHeader.ExtId);
    uint8_t extracted_cmd = Extract_CAN_CMD(rxHeader.ExtId);
    if(extracted_cmd == AWAKE_CMD){
        // awake コマンドを受信した場合
        for(uint8_t i = 0; i < num_detected[can_device.node_type]; i++){
            if(node_id_list[can_device.node_type][i] == rxData[0])return;
        }
        node_id_list[can_device.node_type][num_detected[can_device.node_type]] = rxData[0];
        num_detected[can_device.node_type] += 1;
    }else if(extracted_cmd == FB_CMD){
        // feedback コマンドを受信した場合
        if(can_device.node_type == NODE_MCMD4){
            memcpy(&(_feedback_table_mcmd4[can_device.node_id].feedback_motor[can_device.device_num]), rxData, sizeof(MCMD_Feedback_Typedef));
        }else if(can_device.node_type == NODE_DYNAMIXEL_KONDO){
            memcpy(&(_feedback_table_dynamixel_kondo[can_device.node_id].feedback_dynamixel_kondo[can_device.device_num]), rxData, sizeof(CANDynamixel_Kondo_Feedback_Typedef));
        }else if(can_device.node_type == NODE_VL53L0X){
            memcpy(&(_feedback_table_vl53l0x[can_device.node_id].feedback_vl53l0x), rxData, sizeof(CANVL53L0X_Feedback_Typedef));
        }else{
            printf("Error\n\r");
        }
    }

    if (num_detected[NODE_SERVO] == num_of->servo && num_detected[NODE_AIR] == num_of->air && num_detected[NODE_MCMD4] == num_of->mcmd4 && num_detected[NODE_DYNAMIXEL_KONDO] == num_of->dynamixel_kondo && num_detected[NODE_VL53L0X] == num_of->vl53l0x){
        all_node_detected = 1;
    }
}

void CANLib_WhenCANRxFifo1MsgPending(CAN_HandleTypeDef *phcan, const NUM_OF_DEVICES *const num_of){ // Fifo1MsgPendingで呼び出すこと. CAN受信時に呼び出される関数
    if(phcan != _canlib_phcan)return;
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    if (HAL_CAN_GetRxMessage(_canlib_phcan, CAN_RX_FIFO1, &rxHeader, rxData) != HAL_OK){
        // Reception Error
        printf("GetRxMessage error (FIFO1)\n\r");
        Error_Handler();
    }

    CAN_Device can_device = Extract_CAN_Device(rxHeader.ExtId);
    uint8_t extracted_cmd = Extract_CAN_CMD(rxHeader.ExtId);
    if(extracted_cmd == AWAKE_CMD){
        // awake コマンドを受信した場合
        for(uint8_t i = 0; i < num_detected[can_device.node_type]; i++){
            if(node_id_list[can_device.node_type][i] == rxData[0])return;
        }
        node_id_list[can_device.node_type][num_detected[can_device.node_type]] = rxData[0];
        num_detected[can_device.node_type] += 1;
    }else if(extracted_cmd == FB_CMD){
        // feedback コマンドを受信した場合
        if(can_device.node_type == NODE_MCMD4){
            memcpy(&(_feedback_table_mcmd4[can_device.node_id].feedback_motor[can_device.device_num]), rxData, sizeof(MCMD_Feedback_Typedef));
        }else if(can_device.node_type == NODE_DYNAMIXEL_KONDO){
            memcpy(&(_feedback_table_dynamixel_kondo[can_device.node_id].feedback_dynamixel_kondo[can_device.device_num]), rxData, sizeof(CANDynamixel_Kondo_Feedback_Typedef));
        }else if(can_device.node_type == NODE_VL53L0X){
            memcpy(&(_feedback_table_vl53l0x[can_device.node_id].feedback_vl53l0x), rxData, sizeof(CANVL53L0X_Feedback_Typedef));
        }else{
            printf("Error\n\r");
        }
    }

    if (num_detected[NODE_SERVO] == num_of->servo && num_detected[NODE_AIR] == num_of->air && num_detected[NODE_MCMD4] == num_of->mcmd4 && num_detected[NODE_DYNAMIXEL_KONDO] == num_of->dynamixel_kondo && num_detected[NODE_VL53L0X] == num_of->vl53l0x){
        all_node_detected = 1;
    }
}

/**
 * @brief CANのシステムをアクティベートする. 具体的にはメッセージフィルターの設定とhal_can_start.
 *
 * @param _hcan
 * @param can_param
 */
void CAN_SystemInit(CAN_HandleTypeDef *const _hcan){ // CANの初期化
    CAN_FilterTypeDef sFilterConfig;
    _canlib_phcan = _hcan;

    all_node_detected = 0;
    for (uint8_t type = 0; type < NODE_TYPES; type++){
        num_detected[type] = 0;
        for (uint8_t i = 0; i < 0b111; i++){
            node_id_list[type][i] = 0xff;
        }
    }

    //フィルタバンク設定
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    // CAN2をつかうならSlaveStartFilterBank以上の値をFilterBankに設定する必要がある (27まで)
    // CAN3を使う場合はFilterBankは13までっぽい (stm32f7xx_hal_can.c参照)
    if (_hcan->Instance == CAN1 || _hcan->Instance == CAN3){
        // CAN1 or 3

        // FIFO0に初期化用のフィルタを設定
        sFilterConfig.FilterBank = 0;
        sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_MCMD4, 0, 0, MCMD_CMD_AWAKE) >> 13; // 上16bit
        sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(NODE_MCMD4, 0, 0, 0b11111) >> 13;
        sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_MCMD4, 0, 0, MCMD_CMD_AWAKE) & 0x1FFF) << 3 | 0b100; // 下16bit
        sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(NODE_MCMD4, 0, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
        if (HAL_CAN_ConfigFilter(_canlib_phcan, &sFilterConfig) != HAL_OK){
            /* Filter configuration Error */
            Error_Handler();
        }

        // FIFO0に初期化用のフィルタを設定
        sFilterConfig.FilterBank = 1;
        sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_SERVO, 0, 0, SERVO_CMD_AWAKE) >> 13; // 上16bit
        sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(NODE_SERVO, 0, 0, 0b11111) >> 13;
        sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_SERVO, 0, 0, SERVO_CMD_AWAKE) & 0x1FFF) << 3 | 0b100; // 下16bit
        sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(NODE_SERVO, 0, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
        if (HAL_CAN_ConfigFilter(_canlib_phcan, &sFilterConfig) != HAL_OK){
            /* Filter configuration Error */
            Error_Handler();
        }

        // FIFO0に初期化用のフィルタを設定
        sFilterConfig.FilterBank = 2;
        sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_AIR, 0, 0, AIR_CMD_AWAKE) >> 13; // 上16bit
        sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(NODE_AIR, 0, 0, 0b11111) >> 13;
        sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_AIR, 0, 0, AIR_CMD_AWAKE) & 0x1FFF) << 3 | 0b100; // 下16bit
        sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(NODE_AIR, 0, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
        if (HAL_CAN_ConfigFilter(_canlib_phcan, &sFilterConfig) != HAL_OK){
            /* Filter configuration Error */
            Error_Handler();
        }

        // FIFO0に初期化用のフィルタを設定
        sFilterConfig.FilterBank = 3;
        sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_DYNAMIXEL_KONDO, 0, 0, DYNAMIXEL_KONDO_CMD_AWAKE) >> 13; // 上16bit
        sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(NODE_DYNAMIXEL_KONDO, 0, 0, 0b11111) >> 13;
        sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_DYNAMIXEL_KONDO, 0, 0, DYNAMIXEL_KONDO_CMD_AWAKE) & 0x1FFF) << 3 | 0b100; // 下16bit
        sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(NODE_DYNAMIXEL_KONDO, 0, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
        if (HAL_CAN_ConfigFilter(_canlib_phcan, &sFilterConfig) != HAL_OK){
            /* Filter configuration Error */
            Error_Handler();
        }

        // FIFO0に初期化用のフィルタを設定
        sFilterConfig.FilterBank = 8;
        sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_VL53L0X, 0, 0, VL53L0X_CMD_AWAKE) >> 13; // 上16bit
        sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(NODE_VL53L0X, 0, 0, 0b11111) >> 13;
        sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_VL53L0X, 0, 0, VL53L0X_CMD_AWAKE) & 0x1FFF) << 3 | 0b100; // 下16bit
        sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(NODE_VL53L0X, 0, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
        if (HAL_CAN_ConfigFilter(_canlib_phcan, &sFilterConfig) != HAL_OK){
            /* Filter configuration Error */
            Error_Handler();
        }

        // FIFO1にfeedback用のフィルタを設定
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
        sFilterConfig.FilterBank = 4;
        sFilterConfig.FilterIdHigh = Make_CAN_ID(0, 0, 0, MCMD_CMD_FB) >> 13; // 上16bit
        sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0, 0, 0, 0b11111) >> 13;
        sFilterConfig.FilterIdLow = (Make_CAN_ID(0, 0, 0, MCMD_CMD_FB) & 0x1FFF) << 3 | 0b100; // 下16bit
        sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0, 0, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
        if (HAL_CAN_ConfigFilter(_canlib_phcan, &sFilterConfig) != HAL_OK){
            /* Filter configuration Error */
            Error_Handler();
        }

        // FIFO1にfeedback用のフィルタを設定
        sFilterConfig.FilterBank = 5;
        sFilterConfig.FilterIdHigh = Make_CAN_ID(0, 0, 0, DYNAMIXEL_KONDO_CMD_FB) >> 13; // 上16bit
        sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0, 0, 0, 0b11111) >> 13;
        sFilterConfig.FilterIdLow = (Make_CAN_ID(0, 0, 0, DYNAMIXEL_KONDO_CMD_FB) & 0x1FFF) << 3 | 0b100; // 下16bit
        sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0, 0, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
        if (HAL_CAN_ConfigFilter(_canlib_phcan, &sFilterConfig) != HAL_OK){
            /* Filter configuration Error */
            Error_Handler();
        }

        // FIFO1にfeedback用のフィルタを設定
        sFilterConfig.FilterBank = 9;
        sFilterConfig.FilterIdHigh = Make_CAN_ID(0, 0, 0, VL53L0X_CMD_FB) >> 13; // 上16bit
        sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0, 0, 0, 0b11111) >> 13;
        sFilterConfig.FilterIdLow = (Make_CAN_ID(0, 0, 0, VL53L0X_CMD_FB) & 0x1FFF) << 3 | 0b100; // 下16bit
        sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0, 0, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
        if (HAL_CAN_ConfigFilter(_canlib_phcan, &sFilterConfig) != HAL_OK){
            /* Filter configuration Error */
            Error_Handler();
        }

    }else if (_hcan->Instance == CAN2){
        // CAN2

        // FIFO0に初期化用のフィルタを設定
        sFilterConfig.FilterBank = 15;
        sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_MCMD4, 0, 0, MCMD_CMD_AWAKE) >> 13; // 上16bit
        sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(NODE_MCMD4, 0, 0, 0b11111) >> 13;
        sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_MCMD4, 0, 0, MCMD_CMD_AWAKE) & 0x1FFF) << 3 | 0b100; // 下16bit
        sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(NODE_MCMD4, 0, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
        if (HAL_CAN_ConfigFilter(_canlib_phcan, &sFilterConfig) != HAL_OK){
            /* Filter configuration Error */
            Error_Handler();
        }

        // FIFO0に初期化用のフィルタを設定
        sFilterConfig.FilterBank = 16;
        sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_SERVO, 0, 0, SERVO_CMD_AWAKE) >> 13; // 上16bit
        sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(NODE_SERVO, 0, 0, 0b11111) >> 13;
        sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_SERVO, 0, 0, SERVO_CMD_AWAKE) & 0x1FFF) << 3 | 0b100; // 下16bit
        sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(NODE_SERVO, 0, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
        if (HAL_CAN_ConfigFilter(_canlib_phcan, &sFilterConfig) != HAL_OK){
            /* Filter configuration Error */
            Error_Handler();
        }

        // FIFO0に初期化用のフィルタを設定
        sFilterConfig.FilterBank = 17;
        sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_AIR, 0, 0, AIR_CMD_AWAKE) >> 13; // 上16bit
        sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(NODE_AIR, 0, 0, 0b11111) >> 13;
        sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_AIR, 0, 0, AIR_CMD_AWAKE) & 0x1FFF) << 3 | 0b100; // 下16bit
        sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(NODE_AIR, 0, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
        if (HAL_CAN_ConfigFilter(_canlib_phcan, &sFilterConfig) != HAL_OK){
            /* Filter configuration Error */
            Error_Handler();
        }

        // FIFO0に初期化用のフィルタを設定
        sFilterConfig.FilterBank = 18;
        sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_DYNAMIXEL_KONDO, 0, 0, DYNAMIXEL_KONDO_CMD_AWAKE) >> 13; // 上16bit
        sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(NODE_DYNAMIXEL_KONDO, 0, 0, 0b11111) >> 13;
        sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_DYNAMIXEL_KONDO, 0, 0, DYNAMIXEL_KONDO_CMD_AWAKE) & 0x1FFF) << 3 | 0b100; // 下16bit
        sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(NODE_DYNAMIXEL_KONDO, 0, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
        if (HAL_CAN_ConfigFilter(_canlib_phcan, &sFilterConfig) != HAL_OK){
            /* Filter configuration Error */
            Error_Handler();
        }

        // FIFO0に初期化用のフィルタを設定
        sFilterConfig.FilterBank = 23;
        sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_VL53L0X, 0, 0, VL53L0X_CMD_AWAKE) >> 13; // 上16bit
        sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(NODE_VL53L0X, 0, 0, 0b11111) >> 13;
        sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_VL53L0X, 0, 0, VL53L0X_CMD_AWAKE) & 0x1FFF) << 3 | 0b100; // 下16bit
        sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(NODE_VL53L0X, 0, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
        if (HAL_CAN_ConfigFilter(_canlib_phcan, &sFilterConfig) != HAL_OK){
            /* Filter configuration Error */
            Error_Handler();
        }

        // FIFO1にfeedback用のフィルタを設定
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
        sFilterConfig.FilterBank = 19;
        sFilterConfig.FilterIdHigh = Make_CAN_ID(0, 0, 0, MCMD_CMD_FB) >> 13; // 上16bit
        sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0, 0, 0, 0b11111) >> 13;
        sFilterConfig.FilterIdLow = (Make_CAN_ID(0, 0, 0, MCMD_CMD_FB) & 0x1FFF) << 3 | 0b100; // 下16bit
        sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0, 0, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
        if (HAL_CAN_ConfigFilter(_canlib_phcan, &sFilterConfig) != HAL_OK){
            /* Filter configuration Error */
            Error_Handler();
        }

        // FIFO1にfeedback用のフィルタを設定
        sFilterConfig.FilterBank = 20;
        sFilterConfig.FilterIdHigh = Make_CAN_ID(0, 0, 0, DYNAMIXEL_KONDO_CMD_FB) >> 13; // 上16bit
        sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0, 0, 0, 0b11111) >> 13;
        sFilterConfig.FilterIdLow = (Make_CAN_ID(0, 0, 0, DYNAMIXEL_KONDO_CMD_FB) & 0x1FFF) << 3 | 0b100; // 下16bit
        sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0, 0, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
        if (HAL_CAN_ConfigFilter(_canlib_phcan, &sFilterConfig) != HAL_OK){
            /* Filter configuration Error */
            Error_Handler();
        }

        // FIFO1にfeedback用のフィルタを設定
        sFilterConfig.FilterBank = 24;
        sFilterConfig.FilterIdHigh = Make_CAN_ID(0, 0, 0, VL53L0X_CMD_FB) >> 13; // 上16bit
        sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0, 0, 0, 0b11111) >> 13;
        sFilterConfig.FilterIdLow = (Make_CAN_ID(0, 0, 0, VL53L0X_CMD_FB) & 0x1FFF) << 3 | 0b100; // 下16bit
        sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0, 0, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
        if (HAL_CAN_ConfigFilter(_canlib_phcan, &sFilterConfig) != HAL_OK){
            /* Filter configuration Error */
            Error_Handler();
        }
    }

    if (HAL_CAN_Start(_canlib_phcan) != HAL_OK){
        printf(" -> Start Error\n");
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(_canlib_phcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK){
        printf(" -> FIFO0 CAN_Activation error\n\r");
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(_canlib_phcan, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK){
        printf(" -> FIFO1 CAN_Activation error\n\r");
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(_canlib_phcan, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK){
        printf(" -> MAILBOX CAN_Activation error\n\r");
        Error_Handler();
    }
}

/**
 * @brief CANの全デバイスの接続が確認されるまで待つ.
 *
 */
void CAN_WaitConnect(const NUM_OF_DEVICES *num_of, DelayFunction_t f_delay){ // 他のデバイスが接続されるのを待つ
    if(num_of->mcmd4 == 0 && num_of->air == 0 && num_of->servo == 0 && num_of->dynamixel_kondo == 0 && num_of->vl53l0x == 0){
        all_node_detected = 1;
    }
    while (all_node_detected == 0){
        printf("Waiting CAN_NODES Wake Up...\n\r");
        f_delay(500);
    }
    for (uint8_t i = 0; i < num_of->mcmd4; i++)
        printf("MCMD4 No.%d\n\r", node_id_list[NODE_MCMD4][i]);
    for (uint8_t i = 0; i < num_of->servo; i++)
        printf("Servo No.%d\n\r", node_id_list[NODE_SERVO][i]);
    for (uint8_t i = 0; i < num_of->air; i++)
        printf("Air No.%d\n\r", node_id_list[NODE_AIR][i]);
    for (uint8_t i = 0; i < num_of->dynamixel_kondo; i++)
        printf("Dynamixel_Kondo No.%d\n\r", node_id_list[NODE_DYNAMIXEL_KONDO][i]);
    for (uint8_t i = 0; i < num_of->vl53l0x; i++)
        printf("VL53L0X No.%d\n\r", node_id_list[NODE_VL53L0X][i]);
}


//// MCMD
void MCMD_ChangeControl(const MCMD_HandleTypedef *const hmcmd){ // Ctrl typeを変更する.
    float fdata[2];
    fdata[0] = hmcmd->ctrl_param.PID_param.kp;
    fdata[1] = hmcmd->ctrl_param.PID_param.ki;
    uint64_t can_id = Make_CAN_ID_from_CAN_Device(&(hmcmd->device), MCMD_CMD_CHANGE_CTRL1);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&fdata, sizeof(fdata));

    fdata[0] = hmcmd->ctrl_param.PID_param.kd;
    fdata[1] = hmcmd->ctrl_param.accel_limit_size;
    can_id = Make_CAN_ID_from_CAN_Device(&(hmcmd->device), MCMD_CMD_CHANGE_CTRL2);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&fdata, sizeof(fdata));

    fdata[0] = hmcmd->ctrl_param.PID_param.kff;
    fdata[1] = hmcmd->ctrl_param.gravity_compensation_gain;
    can_id = Make_CAN_ID_from_CAN_Device(&(hmcmd->device), MCMD_CMD_CHANGE_CTRL3);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&fdata, sizeof(fdata));

    uint8_t bdata[6];
    bdata[0] = hmcmd->ctrl_param.ctrl_type;
    bdata[1] = hmcmd->ctrl_param.accel_limit;
    bdata[2] = hmcmd->ctrl_param.feedback;
    bdata[3] = hmcmd->ctrl_param.timup_monitor;
    bdata[4] = hmcmd->fb_type;
    bdata[5] = hmcmd->ctrl_param.gravity_compensation; // TODO : new
    can_id = Make_CAN_ID_from_CAN_Device(&(hmcmd->device), MCMD_CMD_CHANGE_CTRL4);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&bdata, sizeof(bdata));
}

void MCMD_init(const MCMD_HandleTypedef *const hmcmd, DelayFunction_t f_delay){
    uint8_t bdata[4];
    bdata[0] = hmcmd->enc_dir;
    bdata[1] = hmcmd->rot_dir;
    bdata[2] = hmcmd->calib;
    bdata[3] = hmcmd->limit_sw_type;
    uint64_t can_id = Make_CAN_ID_from_CAN_Device(&(hmcmd->device), MCMD_CMD_INIT1);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, bdata, sizeof(bdata));

    float fdata[2];
    fdata[0] = hmcmd->offset;
    fdata[1] = hmcmd->calib_duty;
    can_id = Make_CAN_ID_from_CAN_Device(&(hmcmd->device), MCMD_CMD_INIT2);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&fdata, sizeof(fdata));

    fdata[0] = hmcmd->quant_per_unit;
    fdata[1] = 0;
    can_id = Make_CAN_ID_from_CAN_Device(&(hmcmd->device), MCMD_CMD_INIT3);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&fdata, sizeof(fdata));
    f_delay(50); // これないと動かない(なぜ?)
    MCMD_ChangeControl(hmcmd);
}

void MCMD_Calib(const MCMD_HandleTypedef *const hmcmd){
    uint8_t bdata[4];
    const uint64_t can_id = Make_CAN_ID_from_CAN_Device(&(hmcmd->device), MCMD_CMD_CALIB);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, bdata, sizeof(bdata));
}

void MCMD_Wait_For_Calib(const MCMD_HandleTypedef *const hmcmd, DelayFunction_t f_delay){
    while(Get_MCMD_Feedback(&(hmcmd->device)).end_calib!=1){
        f_delay(10);
    }
    printf("Calibration End\r\n");
}

void MCMD_Control_Enable(const MCMD_HandleTypedef *const hmcmd){
    uint8_t bdata[4];
    const uint64_t can_id = Make_CAN_ID_from_CAN_Device(&(hmcmd->device), MCMD_CMD_ENABLE);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, bdata, sizeof(bdata));
}

void MCMD_Control_Disable(const MCMD_HandleTypedef *const hmcmd){
    uint8_t bdata[4];
    const uint64_t can_id = Make_CAN_ID_from_CAN_Device(&(hmcmd->device), MCMD_CMD_DISABLE);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, bdata, sizeof(bdata));
}

void MCMD_SetTarget(const MCMD_HandleTypedef *const hmcmd, const float target){
    const uint64_t can_id = Make_CAN_ID_from_CAN_Device(&(hmcmd->device), MCMD_CMD_SET_TARGET);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&target, sizeof(target));
}

MCMD_Feedback_Typedef Get_MCMD_Feedback(const CAN_Device *const can_device){
    MCMD_Feedback_Typedef ans;
    if (can_device->node_type == NODE_MCMD4){
        ans.fb_type = _feedback_table_mcmd4[(can_device->node_id)].feedback_motor[(can_device->device_num)].fb_type;
        ans.status = _feedback_table_mcmd4[(can_device->node_id)].feedback_motor[(can_device->device_num)].status;
        ans.value = _feedback_table_mcmd4[(can_device->node_id)].feedback_motor[(can_device->device_num)].value;
        ans.end_calib = _feedback_table_mcmd4[(can_device->node_id)].feedback_motor[(can_device->device_num)].end_calib;
        _feedback_table_mcmd4[(can_device->node_id)].feedback_motor[(can_device->device_num)].end_calib=0;
        return ans;
    }else{
        ans.fb_type = 0;
        ans.status = 0;
        ans.value = 0.0f;
        printf("Get_MCMD_Feedback() error\n\r");
        return ans;
    }
}

////servo
void ServoDriver_Init(const CAN_Device *const can_device, const CANServo_Param_Typedef *const param){
    float fdata[2];
    fdata[0] = param->pulse_width_min;
    fdata[1] = param->pulse_width_max;
    uint64_t can_id = Make_CAN_ID_from_CAN_Device(can_device, SERVO_CMD_INIT1);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&fdata, sizeof(fdata));

    fdata[0] = param->pwm_frequency;
    fdata[1] = param->angle_range;
    can_id = Make_CAN_ID_from_CAN_Device(can_device, SERVO_CMD_INIT2);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&fdata, sizeof(fdata));

    fdata[0] = param->angle_offset;
    fdata[1] = 0.0f;
    can_id = Make_CAN_ID_from_CAN_Device(can_device, SERVO_CMD_INIT3);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&fdata, sizeof(fdata));
}

void ServoDriver_SendValue(const CAN_Device *const can_device, const float angle){
    const uint64_t can_id = Make_CAN_ID_from_CAN_Device(can_device, SERVO_CMD_SET_TARGET);
    if (_CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)(&angle), sizeof(float)) != HAL_OK){
        Error_Handler();
    }
}

/* AirCylinderは初期化の際に, ON状態, もしくはOFF状態のどちらにするかは
 * 機構の初期状態に依存するので, 最初にON or OFFを指定してあげないといけない.
 * 従って, Air_Cylinder()とAirCylinder_SendOutput()の中身は実質同じとなっているが,
 * 初期化するという操作をすべてのデバイスに共通のものとして導入するために,
 * 異なる関数名を用いて明示的にしてある.
 * また, AirCylinderの基板の方にも, 最初は必ず初期化処理(AIR_CMD_INIT)が来るものとして定義してある.
 */

////AirCylinder
void AirCylinder_Init(const CAN_Device *const can_device, const Air_PortStatus_Typedef param){
    const uint64_t can_id = Make_CAN_ID_from_CAN_Device(can_device, AIR_CMD_INIT);
    if (_CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)(&param), sizeof(Air_PortStatus_Typedef)) != HAL_OK){
        Error_Handler();
    }
}

void AirCylinder_SendOutput(const CAN_Device *const can_device, const Air_PortStatus_Typedef param){
    const uint64_t can_id = Make_CAN_ID_from_CAN_Device(can_device, AIR_CMD_OUTPUT);
    if (_CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)(&param), sizeof(Air_PortStatus_Typedef)) != HAL_OK){
        Error_Handler();
    }
}

////Dynamixel
void Dynamixel_ChangeControl(const CANDynamixel_HandleTypedef *const hdynamixel){ // 制御パラメータを変更する.
    if(dynamixel_kondo_board_initialized_type[hdynamixel->device.node_id] == DKBOARD_INITIALIZED_AS_KONDO){
        printf("Dynamixel_Kondo Board No. %u has already been initialized as a Kondo Board!\n", hdynamixel->device.node_id);
        return;
    }

    float fdata[2];
    fdata[0] = hdynamixel->ctrl_param.PID_VEL_param.ki;
    fdata[1] = hdynamixel->ctrl_param.PID_VEL_param.kp;
    uint64_t can_id = Make_CAN_ID_from_CAN_Device(&(hdynamixel->device), DYNAMIXEL_KONDO_CMD_CHANGE_CTRL1);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&fdata, sizeof(fdata));

    fdata[0] = hdynamixel->ctrl_param.PID_POS_param.kd;
    fdata[1] = hdynamixel->ctrl_param.PID_POS_param.ki;
    can_id = Make_CAN_ID_from_CAN_Device(&(hdynamixel->device), DYNAMIXEL_KONDO_CMD_CHANGE_CTRL2);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&fdata, sizeof(fdata));

    fdata[0] = hdynamixel->ctrl_param.PID_POS_param.kp;
    fdata[1] = 0.0f;
    can_id = Make_CAN_ID_from_CAN_Device(&(hdynamixel->device), DYNAMIXEL_KONDO_CMD_CHANGE_CTRL3);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&fdata, sizeof(fdata));

    uint8_t bdata[3];
    bdata[0] = hdynamixel->ctrl_param.ctrl_type;
    bdata[1] = hdynamixel->ctrl_param.feedback;
    bdata[2] = hdynamixel->fb_type;
    can_id = Make_CAN_ID_from_CAN_Device(&(hdynamixel->device), DYNAMIXEL_KONDO_CMD_CHANGE_CTRL4);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&bdata, sizeof(bdata));

    fdata[0] = hdynamixel->ctrl_param.profile_acceleration;
    fdata[1] = hdynamixel->ctrl_param.profile_velocity;
    can_id = Make_CAN_ID_from_CAN_Device(&(hdynamixel->device), DYNAMIXEL_KONDO_CMD_CHANGE_CTRL5);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&fdata, sizeof(fdata));
}

void Dynamixel_init(const CANDynamixel_HandleTypedef *const hdynamixel, DelayFunction_t f_delay){
    if(dynamixel_kondo_board_initialized_type[hdynamixel->device.node_id] == DKBOARD_INITIALIZED_AS_KONDO){
        printf("Dynamixel_Kondo Board No. %u has already been initialized as a Kondo Board!\n", hdynamixel->device.node_id);
        return;
    }else{
        dynamixel_kondo_board_initialized_type[hdynamixel->device.node_id] = DKBOARD_INITIALIZED_AS_DYNAMIXEL;
    }

    uint8_t bdata[4];
    bdata[0] = DKBOARD_INITIALIZED_AS_DYNAMIXEL;
    bdata[1] = hdynamixel->model;
    bdata[2] = hdynamixel->id;
    bdata[3] = hdynamixel->rot_dir;
    uint64_t can_id = Make_CAN_ID_from_CAN_Device(&(hdynamixel->device), DYNAMIXEL_KONDO_CMD_INIT1);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&bdata, sizeof(bdata));

    float fdata[1];
    fdata[0] = hdynamixel->quant_per_degree;
    can_id = Make_CAN_ID_from_CAN_Device(&(hdynamixel->device), DYNAMIXEL_KONDO_CMD_INIT2);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&fdata, sizeof(fdata));

    f_delay(50); // これないと動かない(なぜ?) ← MCMD_init()から継承したため詳細不明
    Dynamixel_ChangeControl(hdynamixel);
}

void Dynamixel_Control_Enable(const CANDynamixel_HandleTypedef *const hdynamixel){
    if(dynamixel_kondo_board_initialized_type[hdynamixel->device.node_id] == DKBOARD_INITIALIZED_AS_KONDO){
        printf("Dynamixel_Kondo Board No. %u has already been initialized as a Kondo Board!\n", hdynamixel->device.node_id);
        return;
    }

    uint8_t bdata[4];
    const uint64_t can_id = Make_CAN_ID_from_CAN_Device(&(hdynamixel->device), DYNAMIXEL_KONDO_CMD_ENABLE);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&bdata, sizeof(bdata));
}

void Dynamixel_Control_Disable(const CANDynamixel_HandleTypedef *const hdynamixel){
    if(dynamixel_kondo_board_initialized_type[hdynamixel->device.node_id] == DKBOARD_INITIALIZED_AS_KONDO){
        printf("Dynamixel_Kondo Board No. %u has already been initialized as a Kondo Board!\n", hdynamixel->device.node_id);
        return;
    }

    uint8_t bdata[4];
    const uint64_t can_id = Make_CAN_ID_from_CAN_Device(&(hdynamixel->device), DYNAMIXEL_KONDO_CMD_DISABLE);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&bdata, sizeof(bdata));
}

void Dynamixel_SetTarget(const CANDynamixel_HandleTypedef *const hdynamixel, const float target){
    if(dynamixel_kondo_board_initialized_type[hdynamixel->device.node_id] == DKBOARD_INITIALIZED_AS_KONDO){
        printf("Dynamixel_Kondo Board No. %u has already been initialized as a Kondo Board!\n", hdynamixel->device.node_id);
        return;
    }

    const uint64_t can_id = Make_CAN_ID_from_CAN_Device(&(hdynamixel->device), DYNAMIXEL_KONDO_CMD_SET_TARGET);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&target, sizeof(target));
}

CANDynamixel_Kondo_Feedback_Typedef Get_Dynamixel_Feedback(const CAN_Device *const can_device){
    CANDynamixel_Kondo_Feedback_Typedef ans = {.fb_type = 0, .status = 0, .value = NAN};

    if (can_device->node_type == NODE_DYNAMIXEL_KONDO){
        if(dynamixel_kondo_board_initialized_type[can_device->node_id] == DKBOARD_INITIALIZED_AS_KONDO){
            printf("Dynamixel_Kondo Board No. %u has already been initialized as a Kondo Board!\n", can_device->node_id);
            return ans;
        }

        ans.fb_type = _feedback_table_dynamixel_kondo[(can_device->node_id)].feedback_dynamixel_kondo[(can_device->device_num)].fb_type;
        ans.status = _feedback_table_dynamixel_kondo[(can_device->node_id)].feedback_dynamixel_kondo[(can_device->device_num)].status;
        ans.value = _feedback_table_dynamixel_kondo[(can_device->node_id)].feedback_dynamixel_kondo[(can_device->device_num)].value;
        return ans;
    }else{
        printf("Get_Dynamixel_Feedback() error!\n\r");
        return ans;
    }
}


////Kondo
void Kondo_ChangeControl(const CANKondo_HandleTypedef *const hkondo){ // 制御パラメータを変更する.
    if(dynamixel_kondo_board_initialized_type[hkondo->device.node_id] == DKBOARD_INITIALIZED_AS_DYNAMIXEL){
        printf("Dynamixel_Kondo Board No. %u has already been initialized as a Dynamixel Board!\n", hkondo->device.node_id);
        return;
    }

    float fdata[2];

    uint64_t can_id = Make_CAN_ID_from_CAN_Device(&(hkondo->device), DYNAMIXEL_KONDO_CMD_CHANGE_CTRL1);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&fdata, sizeof(fdata));

    fdata[0] = hkondo->ctrl_param.PID_EXPOS_param.kd;
    fdata[1] = hkondo->ctrl_param.PID_EXPOS_param.ki;
    can_id = Make_CAN_ID_from_CAN_Device(&(hkondo->device), DYNAMIXEL_KONDO_CMD_CHANGE_CTRL2);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&fdata, sizeof(fdata));

    fdata[0] = hkondo->ctrl_param.PID_EXPOS_param.kp;
    can_id = Make_CAN_ID_from_CAN_Device(&(hkondo->device), DYNAMIXEL_KONDO_CMD_CHANGE_CTRL3);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&fdata, sizeof(fdata));

    uint8_t bdata[3];
    bdata[0] = hkondo->ctrl_param.ctrl_type;
    bdata[1] = hkondo->ctrl_param.feedback;
    bdata[2] = hkondo->fb_type;
    can_id = Make_CAN_ID_from_CAN_Device(&(hkondo->device), DYNAMIXEL_KONDO_CMD_CHANGE_CTRL4);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&bdata, sizeof(bdata));

    uint8_t bdata2[1];
    bdata2[0] = hkondo->ctrl_param.trajectory_type;
    can_id = Make_CAN_ID_from_CAN_Device(&(hkondo->device), DYNAMIXEL_KONDO_CMD_CHANGE_CTRL5);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&bdata2, sizeof(bdata2));
}

void Kondo_init(const CANKondo_HandleTypedef *const hkondo, DelayFunction_t f_delay){
    if(dynamixel_kondo_board_initialized_type[hkondo->device.node_id] == DKBOARD_INITIALIZED_AS_DYNAMIXEL){
        printf("Dynamixel_Kondo Board No. %u has already been initialized as a Dynamixel Board!\n", hkondo->device.node_id);
        return;
    }else{
        dynamixel_kondo_board_initialized_type[hkondo->device.node_id] = DKBOARD_INITIALIZED_AS_KONDO;
    }

    uint8_t bdata[2];
    bdata[0] = DKBOARD_INITIALIZED_AS_KONDO;
    bdata[1] = hkondo->id;
    uint64_t can_id = Make_CAN_ID_from_CAN_Device(&(hkondo->device), DYNAMIXEL_KONDO_CMD_INIT1);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&bdata, sizeof(bdata));

    float fdata[1];
    fdata[0] = hkondo->quant_per_degree;
    can_id = Make_CAN_ID_from_CAN_Device(&(hkondo->device), DYNAMIXEL_KONDO_CMD_INIT2);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&fdata, sizeof(fdata));

    f_delay(50); // これないと動かない(なぜ?) ← MCMD_init()から継承したため詳細不明
    Kondo_ChangeControl(hkondo);
}

void Kondo_Control_Enable(const CANKondo_HandleTypedef *const hkondo){
    if(dynamixel_kondo_board_initialized_type[hkondo->device.node_id] == DKBOARD_INITIALIZED_AS_DYNAMIXEL){
        printf("Dynamixel_Kondo Board No. %u has already been initialized as a Dynamixel Board!\n", hkondo->device.node_id);
        return;
    }

    uint8_t bdata[4];
    const uint64_t can_id = Make_CAN_ID_from_CAN_Device(&(hkondo->device), DYNAMIXEL_KONDO_CMD_ENABLE);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&bdata, sizeof(bdata));
}

void Kondo_Control_Disable(const CANKondo_HandleTypedef *const hkondo){
    if(dynamixel_kondo_board_initialized_type[hkondo->device.node_id] == DKBOARD_INITIALIZED_AS_DYNAMIXEL){
        printf("Dynamixel_Kondo Board No. %u has already been initialized as a Dynamixel Board!\n", hkondo->device.node_id);
        return;
    }

    uint8_t bdata[4];
    const uint64_t can_id = Make_CAN_ID_from_CAN_Device(&(hkondo->device), DYNAMIXEL_KONDO_CMD_DISABLE);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&bdata, sizeof(bdata));
}

void Kondo_SetTarget(const CANKondo_HandleTypedef *const hkondo, const float target){
    if(dynamixel_kondo_board_initialized_type[hkondo->device.node_id] == DKBOARD_INITIALIZED_AS_DYNAMIXEL){
        printf("Dynamixel_Kondo Board No. %u has already been initialized as a Dynamixel Board!\n", hkondo->device.node_id);
        return;
    }

    const uint64_t can_id = Make_CAN_ID_from_CAN_Device(&(hkondo->device), DYNAMIXEL_KONDO_CMD_SET_TARGET);

    float fdata[2];
    fdata[0] = target;
    fdata[1] = 0.0f;

    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&fdata, sizeof(fdata));
}

void Kondo_SetTarget_With_Time(const CANKondo_HandleTypedef *const hkondo, const float target, const float time_sec){
    if(dynamixel_kondo_board_initialized_type[hkondo->device.node_id] == DKBOARD_INITIALIZED_AS_DYNAMIXEL){
        printf("Dynamixel_Kondo Board No. %u has already been initialized as a Dynamixel Board!\n", hkondo->device.node_id);
        return;
    }

    const uint64_t can_id = Make_CAN_ID_from_CAN_Device(&(hkondo->device), DYNAMIXEL_KONDO_CMD_SET_TARGET);
    
    float fdata[2];
    fdata[0] = target;
    fdata[1] = time_sec;

    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&fdata, sizeof(fdata));
}

CANDynamixel_Kondo_Feedback_Typedef Get_Kondo_Feedback(const CAN_Device *const can_device){
    CANDynamixel_Kondo_Feedback_Typedef ans = {.fb_type = 0, .status = 0, .value = NAN};

    if (can_device->node_type == NODE_DYNAMIXEL_KONDO){
        if(dynamixel_kondo_board_initialized_type[can_device->node_id] == DKBOARD_INITIALIZED_AS_DYNAMIXEL){
            printf("Dynamixel_Kondo Board No. %u has already been initialized as a Dynamixel Board!\n", can_device->node_id);
            return ans;
        }

        ans.fb_type = _feedback_table_dynamixel_kondo[(can_device->node_id)].feedback_dynamixel_kondo[(can_device->device_num)].fb_type;
        ans.status = _feedback_table_dynamixel_kondo[(can_device->node_id)].feedback_dynamixel_kondo[(can_device->device_num)].status;
        ans.value = _feedback_table_dynamixel_kondo[(can_device->node_id)].feedback_dynamixel_kondo[(can_device->device_num)].value;
        return ans;
    }else{
        printf("Get_Dynamixel_Feedback() error!\n\r");
        return ans;
    }
}

//// VL53L0X
void VL53L0X_ChangeControl(const CANVL53L0X_HandleTypedef *const hvl53l0x){ // 制御パラメータを変更する.
    uint8_t bdata[4];
    const uint64_t can_id = Make_CAN_ID_from_CAN_Device(&(hvl53l0x->device), VL53L0X_CMD_CHANGE_CTRL);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&bdata, sizeof(bdata));
}

void VL53L0X_init(const CANVL53L0X_HandleTypedef *const hvl53l0x){
    uint8_t bdata[4];
    const uint64_t can_id = Make_CAN_ID_from_CAN_Device(&(hvl53l0x->device), VL53L0X_CMD_INIT);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&bdata, sizeof(bdata));

    HAL_Delay(50); // これないと動かない(なぜ?) ← MCMD_init()から継承したため詳細不明
    VL53L0X_ChangeControl(hvl53l0x);
}

void VL53L0X_Control_Enable(const CANVL53L0X_HandleTypedef *const hvl53l0x){
    uint8_t bdata[4];
    const uint64_t can_id = Make_CAN_ID_from_CAN_Device(&(hvl53l0x->device), VL53L0X_CMD_ENABLE);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&bdata, sizeof(bdata));
}

void VL53L0X_Control_Disable(const CANVL53L0X_HandleTypedef *const hvl53l0x){
    uint8_t bdata[4];
    const uint64_t can_id = Make_CAN_ID_from_CAN_Device(&(hvl53l0x->device), VL53L0X_CMD_DISABLE);
    _CANLib_SendBytes(_canlib_phcan, &_canlib_can_buf_ring, can_id, (uint8_t *)&bdata, sizeof(bdata));
}

CANVL53L0X_Feedback_Typedef Get_VL53L0X_Feedback(const CAN_Device *const can_device){
    CANVL53L0X_Feedback_Typedef ans;

    if (can_device->node_type == NODE_VL53L0X){
        ans.status = _feedback_table_vl53l0x[(can_device->node_id)].feedback_vl53l0x.status;
        ans.value = _feedback_table_vl53l0x[(can_device->node_id)].feedback_vl53l0x.value;
        return ans;
    }else{
        ans.status = 0;
        ans.value = 0.0f;
        printf("Get_VL53L0X_Feedback() error!\n\r");
        return ans;
    }
}
/*
 * MCMD_CAN.c
 *
 *  Created on: 10 14, 2021
 *      Author: Emile
 */
#include "MCMD_CAN.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "light_math.h"
#include "pid.h"
#include "Switch.h"

CAN_HandleTypeDef *phcan;
uint8_t node_id;  // nodeのid(基板の番号)
uint8_t initialize_cmd_received = 0;
Node_Type node_type;  // nodeの種類(MCMD4)
Motor_Ctrl_Typedef motor_ctrl_global[2];
// Recorder recorder_motor1;  // TODO : record


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


void InitMotorCom(CAN_HandleTypeDef* hcan, Node_Type _node_type){
    phcan = hcan;

    node_id = GetID();  // node_idを取得(基板番号)
    node_type = _node_type;  // MCMD4を設定
    for(uint8_t i=0; i<2; ++i){  // モーターのパラメータ初期化
        // device setting
        motor_ctrl_global[i].mcmd_param.device.device_num = i;
        motor_ctrl_global[i].mcmd_param.device.node_id = node_id;
        motor_ctrl_global[i].mcmd_param.device.node_type = node_type;
        motor_ctrl_global[i].target_value = 0.0f;
        motor_ctrl_global[i].duty = 0.0f;

        // status setting
        motor_ctrl_global[i].status = MCMD_STATUS_INIT;
        motor_ctrl_global[i].end_calib = 0;
        motor_ctrl_global[i]._is_initialized = 0;
        motor_ctrl_global[i]._start_change_ctrl = 0;
        motor_ctrl_global[i]._finish_change_ctrl = 0; // 0

        // recorder setting
        // init_record(&recorder_motor1, 100.0f);  // TODO : record
    }

    //CAN初期化
    CAN_FilterTypeDef  sFilterConfig;
    printf("InitMotorCom start \n\r");

    // ここからCANのフィルター設定
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // IDMASKモード
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;  // 32bitモード
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterIdHigh = Make_CAN_ID(node_type, node_id, 0, MCMD_CMD_INIT1) >> 13;
    sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0b111, 0b111, 0, 0b11111) >> 13;
    sFilterConfig.FilterIdLow = (Make_CAN_ID(node_type, node_id, 0, MCMD_CMD_INIT1) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0b111, 0b111, 0, 0b11111) & 0x1FFF)<<3 | 0b100;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;  // Fifo0に受信

    if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK){
        /* Filter configuration Error */
        Error_Handler();
    }

    sFilterConfig.FilterBank = 1;
    sFilterConfig.FilterIdHigh = Make_CAN_ID(node_type, node_id, 0, MCMD_CMD_INIT2) >> 13;
    sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0b111, 0b111, 0, 0b11111) >> 13;
    sFilterConfig.FilterIdLow = (Make_CAN_ID(node_type, node_id, 0, MCMD_CMD_INIT2) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0b111, 0b111, 0, 0b11111) & 0x1FFF)<<3 | 0b100;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

    if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK){
        /* Filter configuration Error */
        Error_Handler();
    }

    sFilterConfig.FilterBank = 2;
    sFilterConfig.FilterIdHigh = Make_CAN_ID(node_type, node_id, 0, MCMD_CMD_INIT3) >> 13;
    sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0b111, 0b111, 0, 0b11111) >> 13;
    sFilterConfig.FilterIdLow = (Make_CAN_ID(node_type, node_id, 0, MCMD_CMD_INIT3) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0b111, 0b111, 0, 0b11111) & 0x1FFF)<<3 | 0b100;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

    if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK){
        /* Filter configuration Error */
        Error_Handler();
    }

    sFilterConfig.FilterBank = 3;
    sFilterConfig.FilterIdHigh = Make_CAN_ID(node_type, node_id, 0, MCMD_CMD_CHANGE_CTRL1) >> 13;
    sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0b111, 0b111, 0, 0b11111) >> 13;
    sFilterConfig.FilterIdLow = (Make_CAN_ID(node_type, node_id, 0, MCMD_CMD_CHANGE_CTRL1) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0b111, 0b111, 0, 0b11111) & 0x1FFF)<<3 | 0b100;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

    if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK){
        /* Filter configuration Error */
        Error_Handler();
    }

    sFilterConfig.FilterBank = 4;
    sFilterConfig.FilterIdHigh = Make_CAN_ID(node_type, node_id, 0, MCMD_CMD_CHANGE_CTRL2) >> 13;
    sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0b111, 0b111, 0, 0b11111) >> 13;
    sFilterConfig.FilterIdLow = (Make_CAN_ID(node_type, node_id, 0, MCMD_CMD_CHANGE_CTRL2) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0b111, 0b111, 0, 0b11111) & 0x1FFF)<<3 | 0b100;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

    if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK){
        /* Filter configuration Error */
        Error_Handler();
    }

    sFilterConfig.FilterBank = 5;
    sFilterConfig.FilterIdHigh = Make_CAN_ID(node_type, node_id, 0, MCMD_CMD_CHANGE_CTRL3) >> 13;
    sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0b111, 0b111, 0, 0b11111) >> 13;
    sFilterConfig.FilterIdLow = (Make_CAN_ID(node_type, node_id, 0, MCMD_CMD_CHANGE_CTRL3) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0b111, 0b111, 0, 0b11111) & 0x1FFF)<<3 | 0b100;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

    if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK){
        /* Filter configuration Error */
        Error_Handler();
    }

    // FIFO1 -> FIFO0に変更した
    sFilterConfig.FilterBank = 6;
    sFilterConfig.FilterIdHigh = Make_CAN_ID(node_type, node_id, 0, MCMD_CMD_CALIB) >> 13;
    sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0b111, 0b111, 0, 0b11111) >> 13;
    sFilterConfig.FilterIdLow = (Make_CAN_ID(node_type, node_id, 0, MCMD_CMD_CALIB) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0b111, 0b111, 0, 0b11111) & 0x1FFF)<<3 | 0b100;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;  // Fifo0に受信

    if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK){
        /* Filter configuration Error */
        Error_Handler();
    }

    sFilterConfig.FilterBank = 7;
    sFilterConfig.FilterIdHigh = Make_CAN_ID(node_type, node_id, 0, MCMD_CMD_ENABLE) >> 13;
    sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0b111, 0b111, 0, 0b11111) >> 13;
    sFilterConfig.FilterIdLow = (Make_CAN_ID(node_type, node_id, 0, MCMD_CMD_ENABLE) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0b111, 0b111, 0, 0b11111) & 0x1FFF)<<3 | 0b100;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

    if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK){
        /* Filter configuration Error */
        Error_Handler();
    }

    sFilterConfig.FilterBank = 8;
    sFilterConfig.FilterIdHigh = Make_CAN_ID(node_type, node_id, 0, MCMD_CMD_DISABLE) >> 13;
    sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0b111, 0b111, 0, 0b11111) >> 13;
    sFilterConfig.FilterIdLow = (Make_CAN_ID(node_type, node_id, 0, MCMD_CMD_DISABLE) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0b111, 0b111, 0, 0b11111) & 0x1FFF)<<3 | 0b100;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

    if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK){
        /* Filter configuration Error */
        Error_Handler();
    }

    sFilterConfig.FilterBank = 9;
    sFilterConfig.FilterIdHigh = Make_CAN_ID(node_type, node_id, 0, MCMD_CMD_SET_TARGET) >> 13;
    sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0b111, 0b111, 0, 0b11111) >> 13;
    sFilterConfig.FilterIdLow = (Make_CAN_ID(node_type, node_id, 0, MCMD_CMD_SET_TARGET) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0b111, 0b111, 0, 0b11111) & 0x1FFF)<<3 | 0b100;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

    if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK){
        /* Filter configuration Error */
        Error_Handler();
    }

    sFilterConfig.FilterBank = 10;  // TODO : new
    sFilterConfig.FilterIdHigh = Make_CAN_ID(node_type, node_id, 0, MCMD_CMD_CHANGE_CTRL4) >> 13;
    sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0b111, 0b111, 0, 0b11111) >> 13;
    sFilterConfig.FilterIdLow = (Make_CAN_ID(node_type, node_id, 0, MCMD_CMD_CHANGE_CTRL4) & 0x1FFF) << 3 | 0b100;
    sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0b111, 0b111, 0, 0b11111) & 0x1FFF)<<3 | 0b100;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

    if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK){
        /* Filter configuration Error */
        Error_Handler();
    }
    // ここまでフィルター設定

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


    while((!motor_ctrl_global[0]._is_initialized) && (!motor_ctrl_global[1]._is_initialized)){  // mainから初期化命令が送られて来るまで待機
        if(HAL_CAN_GetError(hcan) == HAL_CAN_ERROR_BOF){
            HAL_CAN_Init(hcan);
        }
        SendBytes(hcan, Make_CAN_ID(node_type, node_id, 0, MCMD_CMD_AWAKE), &node_id, sizeof(uint8_t));
        printf("send awake cmd\n\r");
        HAL_Delay(1000);
    }
    printf("-------motor0-------\n\r");
    CANMotor_Print(&motor_ctrl_global[0]);
    printf("-------motor1-------\n\r");
    CANMotor_Print(&motor_ctrl_global[1]);
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
            case MCMD_CMD_INIT1:
                if(motor_ctrl_global[device_id]._is_initialized)return HAL_ERROR;
                CANMotor_SetEncDir(&motor_ctrl_global[device_id], buffer[readpoint].rxData[0]);
                CANMotor_SetRotDir(&motor_ctrl_global[device_id], buffer[readpoint].rxData[1]);
                motor_ctrl_global[device_id].mcmd_param.calib=buffer[readpoint].rxData[2];
                motor_ctrl_global[device_id].mcmd_param.limit_sw_type=buffer[readpoint].rxData[3];
                break;
            case MCMD_CMD_INIT2:
                if(motor_ctrl_global[device_id]._is_initialized)return HAL_ERROR;
                memcpy(&fdata, buffer[readpoint].rxData, sizeof(fdata));
                motor_ctrl_global[device_id].mcmd_param.offset = fdata[0];
                motor_ctrl_global[device_id].mcmd_param.calib_duty = fdata[1];
                break;
            case MCMD_CMD_INIT3:
                if(motor_ctrl_global[device_id]._is_initialized)return HAL_ERROR;
                memcpy(&fdata, buffer[readpoint].rxData, sizeof(fdata));
                motor_ctrl_global[device_id].mcmd_param.quant_per_unit = fdata[0];
                motor_ctrl_global[device_id].enc.Init.value_per_pulse = fdata[0];
                initialize_cmd_received = 1;
                break;
            case MCMD_CMD_CHANGE_CTRL1:
                if(!initialize_cmd_received)return HAL_ERROR;
                if(!CANMotor_Change_Status(&motor_ctrl_global[device_id], MCMD_STATUS_CHANGE_CTRL))return HAL_ERROR;
                memcpy(&fdata, buffer[readpoint].rxData, sizeof(fdata));
                motor_ctrl_global[device_id]._start_change_ctrl = 1;
                motor_ctrl_global[device_id]._finish_change_ctrl = 0;
                motor_ctrl_global[device_id].mcmd_param.ctrl_param.PID_param.kp = fdata[0];
                motor_ctrl_global[device_id].mcmd_param.ctrl_param.PID_param.ki = fdata[1];
                break;
            case MCMD_CMD_CHANGE_CTRL2:
                if(!initialize_cmd_received)return HAL_ERROR;
                if(!motor_ctrl_global[device_id]._start_change_ctrl)return HAL_ERROR;
                memcpy(&fdata, buffer[readpoint].rxData, sizeof(fdata));
                motor_ctrl_global[device_id].mcmd_param.ctrl_param.PID_param.kd = fdata[0];
                motor_ctrl_global[device_id].mcmd_param.ctrl_param.accel_limit_size = fdata[1];
                break;
            case MCMD_CMD_CHANGE_CTRL3:
                if(!initialize_cmd_received)return HAL_ERROR;
                if(!motor_ctrl_global[device_id]._start_change_ctrl)return HAL_ERROR;
                memcpy(&fdata, buffer[readpoint].rxData, sizeof(fdata));
                motor_ctrl_global[device_id].mcmd_param.ctrl_param.PID_param.kff = fdata[0]; // TODO : new
                motor_ctrl_global[device_id].mcmd_param.ctrl_param.gravity_compensation_gain = fdata[1]; // TODO : new
                break;
            case MCMD_CMD_CHANGE_CTRL4:
                if(!initialize_cmd_received)return HAL_ERROR;
                if(!motor_ctrl_global[device_id]._start_change_ctrl)return HAL_ERROR;
                motor_ctrl_global[device_id].mcmd_param.ctrl_param.ctrl_type=buffer[readpoint].rxData[0];
                motor_ctrl_global[device_id].mcmd_param.ctrl_param.accel_limit=buffer[readpoint].rxData[1];
                motor_ctrl_global[device_id].mcmd_param.ctrl_param.feedback=buffer[readpoint].rxData[2];
                motor_ctrl_global[device_id].mcmd_param.ctrl_param.timup_monitor=buffer[readpoint].rxData[3];
                motor_ctrl_global[device_id].mcmd_param.fb_type = buffer[readpoint].rxData[4];
                motor_ctrl_global[device_id].mcmd_param.ctrl_param.gravity_compensation = buffer[readpoint].rxData[5]; // TODO : new
                motor_ctrl_global[device_id]._start_change_ctrl = 0;
                motor_ctrl_global[device_id]._finish_change_ctrl = 1;
                motor_ctrl_global[device_id]._is_initialized = 1;  // end init
                break;
            case MCMD_CMD_CALIB:
                if(!CANMotor_Change_Status(&motor_ctrl_global[device_id], MCMD_STATUS_CALIB))return HAL_ERROR;
                break;
            case MCMD_CMD_ENABLE:
                if(!CANMotor_Change_Status(&motor_ctrl_global[device_id], MCMD_STATUS_ENABLE))return HAL_ERROR;
                break;
            case MCMD_CMD_DISABLE:
                if(!CANMotor_Change_Status(&motor_ctrl_global[device_id], MCMD_STATUS_DISABLE))return HAL_ERROR;
                break;
            case MCMD_CMD_SET_TARGET:
                memcpy(&fdata, buffer[readpoint].rxData, sizeof(fdata));
                motor_ctrl_global[device_id].target_value = fdata[0];
                PID_Ctrl_init(&motor_ctrl_global[device_id].mcmd_param.ctrl_param.PID_param);  // pidに必要な情報を初期化
                switch(motor_ctrl_global[device_id].mcmd_param.ctrl_param.ctrl_type){  // prev_valueを現在の制御量にする
                    case MCMD_CTRL_POS:
                        motor_ctrl_global[device_id].mcmd_param.ctrl_param.PID_param.prev_value = motor_ctrl_global[device_id].enc.pos;
                        break;
                    case MCMD_CTRL_VEL:
                        motor_ctrl_global[device_id].mcmd_param.ctrl_param.PID_param.prev_value = motor_ctrl_global[device_id].enc.vel;
                        break;
                    default:
                        break;
                }
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
    if((can_device.node_type != node_type) || (can_device.node_id != node_id)){
        printf("warn get msg to another device\n\r");
        return;  // 別のdeviceへの命令を受信してる
    }
    uint8_t device_id = can_device.device_num;
    PushTx8Bytes(device_id, extracted_cmd, rxData);
}

void WhenCANRxFifo1MsgPending(){//Fifo1MsgPendingで呼び出すこと. CAN受信時に呼び出される
    // 現在は使っていない
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    // Get RX message
    if (HAL_CAN_GetRxMessage(phcan, CAN_RX_FIFO1, &rxHeader, rxData) != HAL_OK){
        // Reception Error
        printf("GetRxMessage error\n\r");
        Error_Handler();
    }
}

uint8_t GetID(){ // 基板のnode idを取得する
    printf("id:%d\n\r", (HAL_GPIO_ReadPin(ID1_GPIO_Port, ID1_Pin)) | (HAL_GPIO_ReadPin(ID2_GPIO_Port, ID2_Pin) << 1) | (HAL_GPIO_ReadPin(ID4_GPIO_Port, ID4_Pin) << 2));
    return (HAL_GPIO_ReadPin(ID1_GPIO_Port, ID1_Pin)) | (HAL_GPIO_ReadPin(ID2_GPIO_Port, ID2_Pin) << 1) | (HAL_GPIO_ReadPin(ID4_GPIO_Port, ID4_Pin) << 2);
}

void CANMotor_Update(Motor_Ctrl_Typedef* can_motor){  // この関数でモーターを回してる
    EncoderUpdateData(&(can_motor->enc));
    // only status_calib, status_enable
    if((can_motor->status) == MCMD_STATUS_CALIB){  // キャリブレーション時の動作
        if(GetSwitchState(can_motor->limit_port, can_motor->limit_pin, can_motor->mcmd_param.limit_sw_type)
                || (can_motor->mcmd_param.calib==CALIBRATION_DISABLE)){
            can_motor->enc.pos=can_motor->mcmd_param.offset;
            can_motor->end_calib = 1;
            PID_Ctrl_init(&(can_motor->mcmd_param.ctrl_param.PID_param)); // PIDパラメータの初期化
        }
        if(can_motor->mcmd_param.calib!=CALIBRATION_DISABLE){
            can_motor->duty = (can_motor->mcmd_param.calib_duty)*(float)(!can_motor->end_calib);
            Motor_Run(&can_motor->motor, (can_motor->duty));
        }
        // Motor_Run : dutyを指定してモーターを回す関数
    }else if((can_motor->status) == MCMD_STATUS_ENABLE){  // MCMDがEnableの時の動作
        float err, duty;
        switch (can_motor->mcmd_param.ctrl_param.ctrl_type){
            case MCMD_CTRL_POS:  // 位置制御
                err=limitf((can_motor->target_value)-(can_motor->enc.pos), can_motor->mcmd_param.ctrl_param.accel_limit_size);
//                can_motor->duty = limitf(PID_Ctrl(&can_motor->mcmd_param.ctrl_param.PID_param, err), 0.7f); // PID Control
                duty = PID_Ctrl(&can_motor->mcmd_param.ctrl_param.PID_param, err, (can_motor->target_value), can_motor->enc.Init.update_freq);
                if((can_motor->mcmd_param.ctrl_param.gravity_compensation) == GRAVITY_COMPENSATION_ENABLE) {
                    duty = duty + (can_motor->enc.pos) * (can_motor->mcmd_param.ctrl_param.gravity_compensation_gain);
                }
                can_motor->duty = limitf(duty, 0.7f); // PID Control
                Motor_Run(&can_motor->motor, (can_motor->duty));
                break;
            case MCMD_CTRL_VEL:  // 速度制御
                err = limitf((can_motor->target_value) - (can_motor->enc.vel),
                                       can_motor->mcmd_param.ctrl_param.accel_limit_size);
//                can_motor->duty = limitf(PID_Ctrl(&can_motor->mcmd_param.ctrl_param.PID_param, err), 0.7f);  // PID Control
                duty = PID_Ctrl(&can_motor->mcmd_param.ctrl_param.PID_param, err, (can_motor->target_value), can_motor->enc.Init.update_freq);
                if((can_motor->mcmd_param.ctrl_param.gravity_compensation) == GRAVITY_COMPENSATION_ENABLE) {
                    duty = duty + (can_motor->enc.vel) * (can_motor->mcmd_param.ctrl_param.gravity_compensation_gain);
                }
                can_motor->duty = limitf(duty, 0.7f); // PID Control
                Motor_Run(&can_motor->motor, (can_motor->duty));
            //    if(can_motor->mcmd_param.device.device_num==1)add_record(&recorder_motor1, can_motor->enc.vel); // TODO : record
                break;
            case MCMD_CTRL_DUTY:  // dutyを決めて回すだけ
                can_motor->duty = limitf(can_motor->target_value, 0.7f);
                Motor_Run(&can_motor->motor, (can_motor->duty));
                // if(can_motor->mcmd_param.device.device_num==1)add_record(&recorder_motor1, can_motor->enc.vel); // TODO : record
                break;
            default:
                break;
        }
    }else if((can_motor->status) == MCMD_STATUS_DISABLE){
        Motor_Run(&can_motor->motor, 0.0f);  // 停止
    }
}

void MCMD_SetTimupFlag(){  // TODO :　timeupは未実装
    for(int i=0;i<2;++i){
        if((motor_ctrl_global[i].status == MCMD_STATUS_ENABLE)
            &&(motor_ctrl_global[i].mcmd_param.ctrl_param.timup_monitor==TIMUP_MONITOR_ENABLE)){
            motor_ctrl_global[i].status = MCMD_STATUS_DISABLE;
        }
    }
}

void CANMotor_SetRotDir(Motor_Ctrl_Typedef* can_motor, MCMD_DIR dir){
    can_motor->mcmd_param.rot_dir=dir;
    if(dir==MCMD_DIR_FW){
        Motor_SetDir(&can_motor->motor, 0); // FW
    }else{
        Motor_SetDir(&can_motor->motor, 1); // BC
    }
}

void CANMotor_SetEncDir(Motor_Ctrl_Typedef* can_motor, MCMD_DIR dir){
    can_motor->mcmd_param.enc_dir=dir;
    can_motor->enc.Init.cnt_dir=dir?-1:1;
}

void CANMotor_Send_Feedback_Main(Motor_Ctrl_Typedef* can_motor){  // MainのマイコンにFeedbackを送信する関数
    if(can_motor->mcmd_param.ctrl_param.feedback == MCMD_FB_DISABLE)return;
    MCMD_Feedback_Typedef tmp;
    tmp.status = can_motor->status;
    tmp.end_calib = can_motor->end_calib;
    switch(can_motor->mcmd_param.fb_type){
        case MCMD_FB_POS:
            tmp.fb_type = MCMD_FB_POS;
            tmp.value = can_motor->enc.pos;
            break;
        case MCMD_FB_VEL:
            tmp.fb_type = MCMD_FB_VEL;
            tmp.value = can_motor->enc.vel;
            break;
        case MCMD_FB_DUTY:
            tmp.fb_type = MCMD_FB_DUTY;
            tmp.value = can_motor->duty;
            break;
        default:
            break;
    }
    if(SendBytes(phcan, Make_CAN_ID_from_CAN_Device(&(can_motor->mcmd_param.device), MCMD_CMD_FB),
                 (uint8_t*)&tmp, sizeof(MCMD_Feedback_Typedef))!=HAL_OK){ // 送信
        Error_Handler();
    }
}

uint8_t CANMotor_Change_Status(Motor_Ctrl_Typedef* can_motor, MCMD_Status next_status){
    MCMD_Status present_status = can_motor->status;  // 現在の状態
    can_motor->incorrect_status_cmd_receive_counter_global += 1;
    switch(next_status){  // ここで状態遷移できるかを判定し, 可能なら遷移している
        case MCMD_STATUS_ENABLE:
            if((present_status !=MCMD_STATUS_CALIB) && (present_status != MCMD_STATUS_DISABLE))return 0;
            if(!(can_motor->end_calib))return 0;  // calibせずに動く事を防止している
            _CANMotor_Enable(can_motor);
            break;
        case MCMD_STATUS_DISABLE:
            if(present_status != MCMD_STATUS_ENABLE)return 0;
            _CANMotor_Disable(can_motor);
            break;
        case MCMD_STATUS_CALIB:
            if((present_status != MCMD_STATUS_DISABLE) && (present_status != MCMD_STATUS_CHANGE_CTRL))return 0;
            if(!can_motor->_finish_change_ctrl)return 0;
            _CANMotor_Calib(can_motor);
            break;
        case MCMD_STATUS_CHANGE_CTRL:
            if((present_status != MCMD_STATUS_INIT) && (present_status != MCMD_STATUS_CHANGE_CTRL)
               && (present_status != MCMD_STATUS_DISABLE))return 0;
            can_motor->target_value = 0.0f;  // change ctrlしたらtarget value(目標値)は0
            can_motor->status = MCMD_STATUS_CHANGE_CTRL;
            can_motor->end_calib = 0;
            break;
        default:
            break;
    }
    can_motor->incorrect_status_cmd_receive_counter_global -= 1;
    return 1;
}

void _CANMotor_Calib(Motor_Ctrl_Typedef* can_motor){
    can_motor->status = MCMD_STATUS_CALIB;
    can_motor->end_calib = 0; // calib終わってないよ
}

void _CANMotor_Enable(Motor_Ctrl_Typedef* can_motor){
    if(can_motor->status == MCMD_STATUS_CALIB) {
        PID_Ctrl_init(&can_motor->mcmd_param.ctrl_param.PID_param);
    }
    can_motor->status = MCMD_STATUS_ENABLE;
}

void _CANMotor_Disable(Motor_Ctrl_Typedef* can_motor){
    can_motor->status = MCMD_STATUS_DISABLE;
}


void CANMotor_Print(Motor_Ctrl_Typedef* can_motor){
    printf("{status=");
    switch(can_motor->status){
        case MCMD_STATUS_INIT:
            printf("init");
            break;
        case MCMD_STATUS_CALIB:
            printf("calib");
            break;
        case MCMD_STATUS_ENABLE:
            printf("enable");
            break;
        case MCMD_STATUS_DISABLE:
            printf("disable");
            break;
        case MCMD_STATUS_CHANGE_CTRL:
            printf("change ctrl");
            break;
        default:
            break;
    }
    printf(",target=%f,", can_motor->target_value);
    switch(can_motor->mcmd_param.ctrl_param.ctrl_type){
        case MCMD_CTRL_POS:
            printf("pos=%f,", can_motor->enc.pos);
            break;
        case MCMD_CTRL_VEL:
            printf("vel=%f,", can_motor->enc.vel);
            break;
        case MCMD_CTRL_DUTY:
            printf("duty mode,");
            break;
        default:
    }
    printf("ac_lim=%f,",can_motor->mcmd_param.ctrl_param.accel_limit_size);
    printf("Kp=%f,",can_motor->mcmd_param.ctrl_param.PID_param.kp);
    printf("fb=%d,",can_motor->mcmd_param.fb_type);
    printf("er_cmd=%d", (int)can_motor->incorrect_status_cmd_receive_counter_global);
    printf("duty=%f}",Motor_GetDuty(&can_motor->motor));
}

// TODO : データ記録用の構造体
void init_record(Recorder* recorder, float magnification){
    recorder->size = 0;
    recorder->id = 1;
    recorder->magnification = magnification;
}

void add_record(Recorder* recorder, float value){
    if(recorder->size >= RECORDER_LENGTH)return;
    recorder->data[(recorder->size)] = (int16_t)(value*(recorder->magnification));
    recorder->size += 1;
}

float get_data_record(Recorder* recorder){
    if(recorder->id >= RECORDER_LENGTH)return recorder->data[RECORDER_LENGTH-1];
    if((recorder->id+1) < recorder->size){
        recorder->id += 1;
        return (float)(recorder->data[(recorder->id)-1])/(recorder->magnification);
    }else{
        return (float)(recorder->data[recorder->id])/(recorder->magnification);
    }
}

uint8_t is_full_record(Recorder* recorder){
    if(recorder->size >= RECORDER_LENGTH)return 1;
    else return 0;
}
// 定義ここまで

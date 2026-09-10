/*
 * CAN_Main_RoboMas.h
 *
 *  Created on: 7 8, 2023
 *      Author: Emile
 */

#ifndef CAN_ROBSTRIDE_SYSTEM_H_
#define CAN_ROBSTRIDE_SYSTEM_H_

// Includes --------------------------------

#include <stdbool.h>

#include "main.h"
#include "CAN_Robstride_Def.h"

// Defines --------------------------------

#define CAN_TXBUFFER_SIZE (512)
#define CAN_PRIORITY_TXBUFFER_SIZE (32U)
#define CAN_TX_MAILBOX_COUNT (3U)

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

/* Service/control frames are kept in a separate non-dropping queue. */
typedef struct {
    CANTxBuf_Robstride buffer[CAN_PRIORITY_TXBUFFER_SIZE];
    uint32_t read_point;
    uint32_t write_point;
    uint8_t is_full;
} CAN_PriorityRingBuf_Robstride;

// Function Prototypes --------------------------------

// Mailbox2を使う
void Robstride_WhenTxMailboxCompleteCallbackCalled(CAN_HandleTypeDef *phcan);

void Robstride_WhenTxMailboxAbortCallbackCalled(CAN_HandleTypeDef *phcan);

void Robstride_WhenCANRxFifo0MsgPending(CAN_HandleTypeDef *phcan);

void Robstride_WhenCANRxFifo1MsgPending(CAN_HandleTypeDef *phcan);

void Get_Robstride_MCUID(const uint8_t rxData[], uint8_t device_id);

void Robstride_SetCANID(Robstride_DeviceInfo *device_info, uint8_t new_id);

void Robstride_ProcessParameter(const uint8_t rxData[], uint8_t device_id);
void Robstride_ProcessParameterFrame(uint32_t ExtID, const uint8_t rxData[], uint8_t device_id);

void Robstride_ProcessFault(const uint8_t rxData[], uint8_t device_id);
void Robstride_CurrentDiagnosticPoll(Robstride_DeviceInfo *device);

HAL_StatusTypeDef Robstride_RequestReadParameter(Robstride_DeviceInfo *device_info,
                                                 uint16_t address);

/* Priority variants are used only by service/control transactions. */
HAL_StatusTypeDef Robstride_RequestReadParameterPriority(Robstride_DeviceInfo *device_info, uint16_t address);

HAL_StatusTypeDef Robstride_WriteFloatData(Robstride_DeviceInfo *device_info,
                                           uint16_t address,
                                           float data);

HAL_StatusTypeDef Robstride_WriteFloatDataPriority(Robstride_DeviceInfo *device_info, uint16_t address, float data);

void Robstride_WriteIntData(Robstride_DeviceInfo *device_info, uint16_t address, int data);

HAL_StatusTypeDef Robstride_WriteIntDataPriority(Robstride_DeviceInfo *device_info, uint16_t address, int data);

HAL_StatusTypeDef Robstride_SendBytes(CAN_HandleTypeDef *phcan, uint8_t motor_id, uint8_t cmd_id, uint16_t option, const uint8_t *bytes, uint32_t size);

/* Service queue operations.  Priority frames are never overwritten. */
HAL_StatusTypeDef Robstride_SendPriorityBytes(CAN_HandleTypeDef *phcan, uint8_t motor_id, uint8_t cmd_id, uint16_t option, const uint8_t *bytes, uint32_t size);
void Robstride_BeginPriorityTransaction(CAN_HandleTypeDef *phcan);
void Robstride_EndPriorityTransaction(CAN_HandleTypeDef *phcan);
void Robstride_ClearPriorityTxQueue(CAN_HandleTypeDef *phcan);
bool Robstride_IsPriorityTransactionActive(const CAN_HandleTypeDef *phcan);

/*
 * Runtime diagnostics.  These counters are read-and-cleared by the
 * application task, so CAN interrupt handlers never print to the UART.
 */
uint32_t Robstride_TakeTxRingOverrunCount(void);
uint32_t Robstride_TakeTxErrorCount(void);
uint32_t Robstride_TakeCanErrorCount(void);
uint32_t Robstride_TakeCanErrorCode(void);
uint32_t Robstride_TakePriorityQueueFullCount(void);

/* 周期試験用。ID1/ID2の順、IRQ内はカウンタ加算だけ行う。 */
typedef struct {
    uint32_t target_submit[2];
    uint32_t target_max_gap_ms[2];
    uint32_t iq_rx[2];
    uint32_t position_rx[2];
    uint32_t velocity_rx[2];
    uint32_t type2_rx[2];
    uint32_t tx_complete;
} Robstride_RateCounters;
Robstride_RateCounters Robstride_TakeRateCounters(void);
/* Type2専用の同一フレーム・スナップショット。将来の位置MPCも共用する。 */
typedef struct { float position, velocity, torque; uint32_t tick; bool valid; } Robstride_StandardFeedback;
bool Robstride_ReadStandardFeedback(const Robstride_DeviceInfo *device, Robstride_StandardFeedback *feedback);
bool Robstride_UsesStandardFeedback(const Robstride_DeviceInfo *device);
/* Type2トルク / 公称トルク定数。Arms相当の参考値で、iqf実測値ではない。 */
float Robstride_StandardReferenceCurrent(const Robstride_DeviceInfo *device, const Robstride_StandardFeedback *feedback);

/* CAN error callbackから呼び出す。送信失敗時もCANキューを止めない。 */
void Robstride_WhenCANErrorCallbackCalled(CAN_HandleTypeDef *phcan);

void Init_Robstride_CAN_System(CAN_HandleTypeDef *phcan);

void Robstride_fb_init(Robstride_DeviceInfo *device_info);

/* Robstrideの位置表現の有効範囲を取得する。 */
bool Robstride_GetPositionLimits(Robstride_device device,
                                 float *min_position,
                                 float *max_position);

/* 機械位置をゼロ化した後などに、位置アンラップの基準を捨てる。 */
void Robstride_ResetPositionTracking(Robstride_DeviceInfo *device_info);

Robstride_FeedbackData Get_Robstride_FeedbackData(Robstride_DeviceInfo *device_info);  // Robstride からデータを読みだして FB を取得
Robstride_FeedbackData Read_Robstride_FeedbackData(Robstride_DeviceInfo *device_info); // 以前取得して保管されている Robstride の FB データを返す（新たに通信を行わない）

/* Fresh-response counters used to validate service completion. */
uint32_t Robstride_GetFeedbackSequence(const Robstride_DeviceInfo *device_info);
/* Type17 mechPos専用。Type2の量子化位置と混ぜず、受信時刻も返す。 */
bool Robstride_ReadMeasuredPosition(const Robstride_DeviceInfo *device, float *position, uint32_t *tick);
uint32_t Robstride_GetParameterSequence(const Robstride_DeviceInfo *device_info, uint16_t address);

#endif

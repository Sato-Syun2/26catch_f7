#ifndef CAN_DEVICES_H
#define CAN_DEVICES_H

#include "CAN_RoboMas.h"
#include "CAN_Robstride.h"
#include "CAN_Main.h"

#define ROBSTRIDE_DEVICE_COUNT 0U
#define ROBOMAS_C610_COUNT 1U
#define ROBOMAS_C620_COUNT 0U
#define ROBOMAS_DEVICE_COUNT (ROBOMAS_C610_COUNT + ROBOMAS_C620_COUNT)

/*
 * Robstride の接続先。1 台ずつ確認する場合は DEVICE_COUNT を 1 にして
 * スロット 0 の種類と CAN ID を切り替える。
 */
#define ROBSTRIDE_DEVICE_0_TYPE Robstride_02
#define ROBSTRIDE_DEVICE_0_ID   2U
#define ROBSTRIDE_DEVICE_1_TYPE Robstride_05_Edu
#define ROBSTRIDE_DEVICE_1_ID   1U

/* C は長さ 0 の配列を標準では許可しないため、未接続時も最小領域を確保する。 */
#define ROBOMAS_C610_DEVICE_0_ID 4U
#define ROBOMAS_C610_DEVICE_1_ID 1U
#define ROBOMAS_C620_DEVICE_0_ID 2U
#define ROBOMAS_C620_DEVICE_1_ID 3U

#define ROBSTRIDE_DEVICE_STORAGE_COUNT \
  ((ROBSTRIDE_DEVICE_COUNT > 0U) ? ROBSTRIDE_DEVICE_COUNT : 1U)
#define ROBOMAS_DEVICE_STORAGE_COUNT \
  ((ROBOMAS_DEVICE_COUNT > 0U) ? ROBOMAS_DEVICE_COUNT : 1U)

extern Robstride_DeviceInfo robstride_dev_info_global[ROBSTRIDE_DEVICE_STORAGE_COUNT];
extern RoboMas_DeviceInfo robomas_dev_info_global[ROBOMAS_DEVICE_STORAGE_COUNT];
extern const NUM_OF_DEVICES num_of_devices;
extern Robstride_FeedbackData feedback_data_raw[ROBSTRIDE_DEVICE_STORAGE_COUNT];
extern float feedback_offset[ROBSTRIDE_DEVICE_STORAGE_COUNT];

void CanDevices_Init(CAN_HandleTypeDef *robomas_can,
                     CAN_HandleTypeDef *robstride_can,
                     DelayFunction_t delay_function);

#endif /* CAN_DEVICES_H */

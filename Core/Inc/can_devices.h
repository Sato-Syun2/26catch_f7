#ifndef CAN_DEVICES_H
#define CAN_DEVICES_H

#include "CAN_RoboMas.h"
#include "CAN_Robstride.h"
#include "CAN_Main.h"

#define ROBSTRIDE_DEVICE_COUNT 2U
#define ROBOMAS_C610_COUNT 2U
#define ROBOMAS_C620_COUNT 2U
#define ROBOMAS_DEVICE_COUNT (ROBOMAS_C610_COUNT + ROBOMAS_C620_COUNT)

extern Robstride_DeviceInfo robstride_dev_info_global[ROBSTRIDE_DEVICE_COUNT];
extern RoboMas_DeviceInfo robomas_dev_info_global[ROBOMAS_DEVICE_COUNT];
extern const NUM_OF_DEVICES num_of_devices;
extern Robstride_FeedbackData feedback_data_raw[ROBSTRIDE_DEVICE_COUNT];
extern float feedback_offset[ROBSTRIDE_DEVICE_COUNT];

void CanDevices_Init(CAN_HandleTypeDef *robomas_can,
                     CAN_HandleTypeDef *robstride_can);

#endif /* CAN_DEVICES_H */

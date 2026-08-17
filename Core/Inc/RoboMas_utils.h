/*
 * RoboMas_utils.h
 *
 *  Created on: Mar 17, 2025
 *      Author: tetud
 */

#ifndef INC_ROBOMAS_UTILS_H_
#define INC_ROBOMAS_UTILS_H_

#include "CAN_RoboMas.h"
#include "CAN_RoboMas_Def.h"
#include "math.h"
#include "stdio.h"

extern RoboMas_DeviceInfo robomas_dev_info_global[];
extern RoboMas_FeedbackData robomas_fb[];

extern const uint8_t num_of_c620;
extern const uint8_t num_of_c610;

extern const uint8_t num_of_robomas;

//extern const uint8_t num_of_c620;
//extern RoboMas_DeviceInfo c620_dev_info_global[8];

extern CAN_HandleTypeDef hcan2;


#endif /* INC_ROBOMAS_UTILS_H_ */

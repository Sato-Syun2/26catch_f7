/*
 * Robstride_utils.h
 *
 *  Created on: Mar 23, 2025
 *      Author: tetud
 */

#ifndef INC_ROBSTRIDE_UTILS_H_
#define INC_ROBSTRIDE_UTILS_H_

#include "CAN_Robstride.h"
#include "CAN_Robstride_Def.h"
#include <stdbool.h>
#include "math.h"
#include "stdio.h"

extern Robstride_DeviceInfo robstride_dev_info_global[];
extern Robstride_FeedbackData robstride_fb[];
extern const uint8_t master_can_id;
extern const uint8_t num_of_robstride;
extern bool robstride_first_flag;
extern volatile float robstride_target_value[];
extern Robstride_FeedbackData feedback_data[];

extern CAN_HandleTypeDef hcan3;

extern float feedback_offset[];
extern volatile float max_angle_limit[2];
extern volatile float min_angle_limit[2];

#endif /* INC_ROBSTRIDE_UTILS_H_ */

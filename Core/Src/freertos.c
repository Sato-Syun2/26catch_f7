/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>

#include "usart.h"
#include "CAN_Main.h"
#include "can_devices.h"
#include "CAN_Robstride.h"
#include "CAN_Robstride_Def.h"
#include "Robstride_utils.h"
#include "CAN_RoboMas.h"
#include "CAN_RoboMas_Def.h"
#include "RoboMas_utils.h"
#include "microros_app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* micro-ROS 本体は Core/Src/microros_app.c に分離している。 */
/* USER CODE END Variables */
osThreadId MROSTaskHandle;
uint32_t defaultTaskBuffer[ 4096 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId RobstrideTaskHandle;
uint32_t RobstrideTaskBuffer[ 256 ];
osStaticThreadDef_t RobstrideTaskControlBlock;
osThreadId RobomasTaskHandle;
uint32_t RobomasTaskBuffer[ 256 ];
osStaticThreadDef_t RobomasTaskControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE END FunctionPrototypes */

void StartMROSTask(void const * argument);
void StartRobstrideTask(void const * argument);
void StartRobomasTask(void const * argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize)
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize)
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add timers, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of MROSTask */
  osThreadStaticDef(MROSTask, StartMROSTask, osPriorityNormal, 0, 4096, defaultTaskBuffer, &defaultTaskControlBlock);
  MROSTaskHandle = osThreadCreate(osThread(MROSTask), NULL);

  /* definition and creation of RobstrideTask */
  osThreadStaticDef(RobstrideTask, StartRobstrideTask, osPriorityIdle, 0, 256, RobstrideTaskBuffer, &RobstrideTaskControlBlock);
  RobstrideTaskHandle = osThreadCreate(osThread(RobstrideTask), NULL);

  /* definition and creation of RobomasTask */
  osThreadStaticDef(RobomasTask, StartRobomasTask, osPriorityIdle, 0, 256, RobomasTaskBuffer, &RobomasTaskControlBlock);
  RobomasTaskHandle = osThreadCreate(osThread(RobomasTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartMROSTask */
/**
  * @brief  Function implementing the MROSTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMROSTask */
void StartMROSTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN StartMROSTask */
  (void)argument;
  MicroRosTask_Run();
  /* USER CODE END StartMROSTask */
}

/* USER CODE BEGIN Header_StartRobstrideTask */
/**
* @brief Function implementing the RobstrideTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRobstrideTask */
void StartRobstrideTask(void const * argument)
{
  /* USER CODE BEGIN StartRobstrideTask */
  (void)argument;
  for (;;) {
    /* ROS から受信した最新指令を CAN の周期に合わせて再送する。 */
    MicroRos_RefreshRobstrideTargets();

    for (uint8_t i = 0U; i < ROBSTRIDE_DEVICE_COUNT; ++i) {
      feedback_data[i] = Get_Robstride_FeedbackData(&robstride_dev_info_global[i]);
    }

    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
    osDelay(10U);
  }
  /* USER CODE END StartRobstrideTask */
}

/* USER CODE BEGIN Header_StartRobomasTask */
/**
* @brief Function implementing the RobomasTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRobomasTask */
void StartRobomasTask(void const * argument)
{
  /* USER CODE BEGIN StartRobomasTask */
#if ROBOMAS_C610_COUNT > 0U
  printf("Calibration...\r\n");
  RoboMas_Calibration(&robomas_dev_info_global[0],
                      -3.0f,
                      ROBOMAS_SWITCH_NO,
                      sensor1_GPIO_Port,
                      sensor1_Pin,
                      &hcan2);
  printf("Calibration done.\r\n");
#endif

  (void)argument;
  for (;;) {
#if ROBOMAS_DEVICE_COUNT > 0U
    RoboMas_SendRequest(robomas_dev_info_global,
                        num_of_robomas,
                        500.0f,
                        &hcan2);
    for (uint8_t i = 0U; i < num_of_robomas; ++i) {
      robomas_fb[i] = Get_RoboMas_FeedbackData(&robomas_dev_info_global[i]);
    }
#endif
    osDelay(2U);
  }
  /* USER CODE END StartRobomasTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* micro-ROS の topic 処理は Core/Src/microros_app.c に分離している。 */
/* USER CODE END Application */


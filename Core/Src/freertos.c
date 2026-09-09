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

#include "can.h"
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
osThreadId CanDevicesTaskHandle;
uint32_t CanDevicesTaskBuffer[1024];
osStaticThreadDef_t CanDevicesTaskControlBlock;
osThreadId RobstrideTaskHandle;
uint32_t RobstrideTaskBuffer[ 256 ];
osStaticThreadDef_t RobstrideTaskControlBlock;
osThreadId RobomasTaskHandle;
uint32_t RobomasTaskBuffer[ 256 ];
osStaticThreadDef_t RobomasTaskControlBlock;

/*
 * MX_LWIP_Init() creates the Ethernet/LwIP objects and enables the ETH
 * receive path. Keep motor post-initialization behind this barrier so a
 * link-up packet cannot arrive while the CAN device tasks are starting.
 */
static volatile bool ethernet_init_complete = false;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE END FunctionPrototypes */

void StartMROSTask(void const *argument);
void StartCanDevicesTask(void const *argument);
void StartRobstrideTask(void const *argument);
void StartRobomasTask(void const *argument);

/*
 * Robstride_WaitForConnect() is blocking by design, but it must yield when
 * called from a FreeRTOS task.  HAL_Delay() busy-waits on this target and can
 * starve the lower-priority Ethernet link task while a motor is absent.
 */
static void CanDevices_RtosDelay(uint32_t milliseconds)
{
  (void)osDelay(milliseconds == 0U ? 1U : milliseconds);
}

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
  ethernet_init_complete = false;
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

  osThreadStaticDef(CanDevicesTask,
                    StartCanDevicesTask,
                    osPriorityNormal,
                    0,
                    1024,
                    CanDevicesTaskBuffer,
                    &CanDevicesTaskControlBlock);
  CanDevicesTaskHandle = osThreadCreate(osThread(CanDevicesTask), NULL);


  osThreadStaticDef(RobstrideTask,
                    StartRobstrideTask,
                    /* ROS過負荷時にもCAN制御・診断タスクを止めない。 */
                    osPriorityAboveNormal,
                    0,
                    256,
                    RobstrideTaskBuffer,
                    &RobstrideTaskControlBlock);
  RobstrideTaskHandle = osThreadCreate(osThread(RobstrideTask), NULL);

  osThreadStaticDef(RobomasTask,
                    StartRobomasTask,
                    /* Motor control must not be starved by Ethernet traffic. */
                    osPriorityAboveNormal,
                    0,
                    256,
                    RobomasTaskBuffer,
                    &RobomasTaskControlBlock);
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
  (void)argument;

  /* init code for LWIP */
  MX_LWIP_Init();

  /* Publish the barrier only after all LwIP/ETH objects have been created. */
  __DMB();
  ethernet_init_complete = true;

  /* USER CODE BEGIN StartMROSTask */
  MicroRosTask_Run();
  /* USER CODE END StartMROSTask */
}

/* USER CODE BEGIN Header_StartCanDevicesTask */
/**
* @brief Run the existing blocking motor connection wait after the scheduler
*        has started, so it cannot prevent micro-ROS from coming up.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanDevicesTask */
void StartCanDevicesTask(void const *argument)
{
  /* USER CODE BEGIN StartCanDevicesTask */
  (void)argument;

  /* Do not overlap CAN motor setup with LwIP/ETH creation. */
  while (!ethernet_init_complete) {
    osDelay(10U);
  }

  CanDevices_InitAfterWait(CanDevices_RtosDelay);
  for (;;) {
    osDelay(1000U);
  }
  /* USER CODE END StartCanDevicesTask */
}

#if ROBOMAS_DEVICE_COUNT > 0U
/* フィードバックが準備できたモーターだけを個別にキャリブレーションする。 */
static void start_robomas_calibration_if_ready(
    bool calibration_started[ROBOMAS_DEVICE_STORAGE_COUNT])
{
#if ROBOMAS_C610_COUNT > 0U
  if (!calibration_started[0] && robomas_fb[0].get_flag != 0U) {
    printf("[RoboMas] ID %u feedback ready; calibration start\r\n",
           (unsigned int)robomas_dev_info_global[0].device_id);
    RoboMas_Calibration(&robomas_dev_info_global[0],
                        -40.0f,
                        ROBOMAS_SWITCH_NO,
                        sensor2_GPIO_Port,
                        sensor2_Pin,
                        &hcan2);
    calibration_started[0] = true;
  }
#endif

#if ROBOMAS_C610_COUNT > 1U
  /* ID4単独試験中はID1を起動時にも動かさない。ゲイン設定は維持する。 */
  const bool id4_only_test = true;
  if (!id4_only_test && !calibration_started[1] && robomas_fb[1].get_flag != 0U) {
    printf("[RoboMas] ID %u feedback ready; calibration start\r\n",
           (unsigned int)robomas_dev_info_global[1].device_id);
    RoboMas_Calibration(&robomas_dev_info_global[1],
                        -40.0f,
                        ROBOMAS_SWITCH_NO,
                        sensor1_GPIO_Port,
                        sensor1_Pin,
                        &hcan2);
    calibration_started[1] = true;
  }
#endif
}
#endif

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

  /* CanDevicesTask owns the blocking connection wait and final setup. */
  while (!CanDevices_IsInitialized()) {
    osDelay(10U);
  }

  uint8_t feedback_divider = 0U;
  for (;;) {
    MicroRos_CheckRobstrideCommandTimeout();

    /* ROS受信とCAN送信を分離し、ここをRobstrideの制御周期にする。 */
    MicroRos_ApplyPendingRobstrideCommands();
    MicroRos_ReportDiagnostics();

    /* 目標値の入力はSetTarget()へ集約し、VEL_DOBも同じ経路で500 Hz実行する。 */
    MicroRos_RefreshRobstrideTargets();

    /*
     * 既存のGet経路を意図的に維持し、100 Hzで各モーターの
     * Type 17パラメータ要求を発行する。サービス通信は別の優先キュー
     * と送信確認経路を通るため、この過負荷条件でも応答を混同しない。
     */
    ++feedback_divider;
    if (feedback_divider >= 5U) {
      for (uint8_t i = 0U; i < ROBSTRIDE_DEVICE_COUNT; ++i) {
        feedback_data[i] = Get_Robstride_FeedbackData(
            &robstride_dev_info_global[i]);
      }
      feedback_divider = 0U;
    }

    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
    osDelay(2U);
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
  bool calibration_started[ROBOMAS_DEVICE_STORAGE_COUNT] = {false};
#if ROBOMAS_DEVICE_COUNT > 0U
  bool robomas_feedback_wait_logged[ROBOMAS_DEVICE_STORAGE_COUNT] = {false};
#endif
#if ROBOMAS_C610_COUNT > 0U
  bool calibration_first_done_printed = false;
#if ROBOMAS_C610_COUNT > 1U
  bool calibration_second_done_printed = false;
#endif
#endif

  (void)argument;

  TickType_t robomas_last_wake_time = xTaskGetTickCount();

#if ROBOMAS_DEVICE_COUNT > 0U
  printf("[RoboMas] task started; prepared=%u ethernet=%u\r\n",
         (unsigned)CanDevices_IsPrepared(),
         (unsigned)ethernet_init_complete);
#endif

  for (;;) {
#if ROBOMAS_DEVICE_COUNT > 0U
    /* 初期化未完了時もタスクを止めず、安全な状態で次周期へ進む。 */
    if (CanDevices_IsPrepared()) {
      MicroRos_CheckRobomasCommandTimeout();

      /* 接続待ちを行わず、受信済みフィードバックを毎周期更新する。 */
      for (uint8_t i = 0U; i < num_of_robomas; ++i) {
        const RoboMas_FeedbackData feedback =
            Get_RoboMas_FeedbackData(&robomas_dev_info_global[i]);
        robomas_fb[i] = feedback;
        if ((feedback.get_flag == 0U) &&
            !robomas_feedback_wait_logged[i]) {
          printf("[RoboMas] ID %u waiting for CAN feedback\r\n",
                 (unsigned)robomas_dev_info_global[i].device_id);
          robomas_feedback_wait_logged[i] = true;
        }
      }

      /* Ethernet初期化完了後、準備できたモーターから順に開始する。 */
      if (ethernet_init_complete) {
        start_robomas_calibration_if_ready(calibration_started);
      }

      RoboMas_SendRequest(robomas_dev_info_global,
                          num_of_robomas,
                          500.0f,
                          &hcan2);

#if ROBOMAS_C610_COUNT > 0U
    /* 各モーターの完了を個別に通知する。 */
    if (!calibration_first_done_printed &&
        RoboMas_IsCalibrationEnded(&robomas_dev_info_global[0])) {
      printf("Calibration 1 done.\r\n");
      calibration_first_done_printed = true;
    }

#if ROBOMAS_C610_COUNT > 1U
    if (!calibration_second_done_printed &&
        RoboMas_IsCalibrationEnded(&robomas_dev_info_global[1])) {
      printf("Calibration 2 done.\r\n");
      calibration_second_done_printed = true;
    }
#endif
#endif
    }
#endif
    /* 未接続モーターがあっても周期処理を止めない。 */
    vTaskDelayUntil(&robomas_last_wake_time, pdMS_TO_TICKS(2U));
  }
  /* USER CODE END StartRobomasTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* micro-ROS の topic 処理は Core/Src/microros_app.c に分離している。 */
/* USER CODE END Application */

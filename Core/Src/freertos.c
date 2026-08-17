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
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include "usart.h"
#include "lwip/netif.h"
#include "CAN_Main.h"

#include "CAN_Robstride.h"
#include "CAN_Robstride_Def.h"
#include "Robstride_utils.h"

#include "CAN_RoboMas.h"
#include "CAN_RoboMas_Def.h"
#include "RoboMas_utils.h"

#include <geometry_msgs/msg/point.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int8_multi_array.h>
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

/* micro-ROS のエンティティと受信・送信メッセージをまとめて保持する。 */
static rcl_subscription_t point_subscriber;
static rcl_publisher_t robstride_fb_publisher;
static geometry_msgs__msg__Point point_msg;

/* 外部の UDP トランスポート／FreeRTOS アロケータ実装の宣言。 */
bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport,
                              const uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read(struct uxrCustomTransport *transport,
                             uint8_t *buf, size_t len, int timeout, uint8_t *err);
void *microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void *microros_reallocate(void *pointer, size_t size, void *state);
void *microros_zero_allocate(size_t number_of_elements, size_t size_of_element,
                             void *state);

/* 初期化失敗時にエラーを表示し、以降の状態を調査できるようにする。 */
#define RCCHECK(call) do { \
  rcl_ret_t rcl_ret = (call); \
  if (rcl_ret != RCL_RET_OK) { \
    printf("micro-ROS error: %d (line %d)\\r\\n", (int)rcl_ret, __LINE__); \
    rcl_reset_error(); \
  } \
} while (0)

/* USER CODE END Variables */
osThreadId MROSTaskHandle;
uint32_t defaultTaskBuffer[ 1024 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId RobstrideTaskHandle;
uint32_t RobstrideTaskBuffer[ 256 ];
osStaticThreadDef_t RobstrideTaskControlBlock;
osThreadId RobomasTaskHandle;
uint32_t RobomasTaskBuffer[ 256 ];
osStaticThreadDef_t RobomasTaskControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static void point_callback(const void *msgin);
static void feedback_timer_callback(rcl_timer_t *timer, int64_t last_call_time);
static void subscription_callback(const void *msgin);

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

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
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

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
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
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of MROSTask */
  osThreadStaticDef(MROSTask, StartMROSTask, osPriorityNormal, 0, 1024, defaultTaskBuffer, &defaultTaskControlBlock);
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
  printf("Start Micro-ROS Task\r\n");
  /* Infinite loop */
  struct netif *netif_ptr = netif_default;
  while (netif_ptr == NULL || netif_ptr->ip_addr.addr == 0) {
    osDelay(100);
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7); // Toggle LD3 (RED) for debug
    printf("Waiting for valid IP address...\r\n");
  }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); // LD3 (GREEN) -> ON

  osDelay(1000);
  printf("Link is not up yet0!\r\n");
  extern struct netif gnetif;
  while (!netif_is_link_up(&gnetif)) {
      printf("Link is not up yet!\r\n");
      osDelay(1000);
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0); //for debug
  }
  printf("Link is up!\r\n");

  for(;;)
  {
	    const char* agent_ip = "192.168.4.100";
			rmw_uros_set_custom_transport(
			  false,                 // UDP を使う
			  (void *) agent_ip,   // Agent IP address
			  cubemx_transport_open,
			  cubemx_transport_close,
			  cubemx_transport_write,
			  cubemx_transport_read
			);

	  	// micro-ROS connection check
	  	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);  // LD3 (RED) -> ON
	  	while(1) {
//	  		printf("tes\n\r");
	  		rmw_ret_t ping_result = rmw_uros_ping_agent(1000, 5);  // ping Agent
//	  		printf("test\n\r");
	  		if(ping_result == RMW_RET_OK){
	  			break;
	  		}
	  	}
	  	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);  // LD3 (RED) -> OFF

		rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
		freeRTOS_allocator.allocate = microros_allocate;
		freeRTOS_allocator.deallocate = microros_deallocate;
		freeRTOS_allocator.reallocate = microros_reallocate;
		freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

		if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
			printf("Error on default allocators (line %d)\r\n", __LINE__);
		}
		printf("start Micro-ROS Task\r\n");

		// micro-ROS app
		setvbuf(stdout, NULL, _IOFBF, BUFSIZ);
		rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
		rclc_support_t support;
		rcl_allocator_t allocator = rcl_get_default_allocator();
		rcl_node_t node;
		rcl_node_options_t node_ops = rcl_node_get_default_options();

		// node setting
		RCCHECK(rcl_init_options_init(&init_options, allocator));
		RCCHECK(rcl_init_options_set_domain_id(&init_options, 30)); // ROS_DOMAIN_IDの設定。今回は30としてる。
		rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
		RCCHECK(rclc_node_init_with_options(&node, "f7_mros_node", "", &support, &node_ops));

		// create executor
		rclc_executor_t executor;
		unsigned int num_handlers = 3;// TODO : 忘れずに変更
       RCCHECK(rclc_executor_init(&executor, &support.context, num_handlers, &allocator));

       const char* point_sub_name = "mros_point_input";
       rmw_qos_profile_t point_sub_options = rmw_qos_profile_default;
       point_sub_options.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
       RCCHECK(rclc_subscription_init(&point_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), point_sub_name, &point_sub_options));
       RCCHECK(rclc_executor_add_subscription(&executor, &point_subscriber, &point_msg, &point_callback, ON_NEW_DATA));

       const char* robstride_pub_name = "robstride_feedback_pub";
       RCCHECK(rclc_publisher_init_default(&robstride_fb_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), robstride_pub_name));
       rcl_timer_t feedback_timer;
       RCCHECK(rclc_timer_init_default(&feedback_timer, &support, RCL_MS_TO_NS(10), feedback_timer_callback));
       RCCHECK(rclc_executor_add_timer(&executor, &feedback_timer));

		rcl_subscription_t subscriber;
		const char* sub_name = "mros_input_uint8";
		std_msgs__msg__UInt8 msg;
		rmw_qos_profile_t sub_options = rmw_qos_profile_default;
		sub_options.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
		RCCHECK(rclc_subscription_init(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), sub_name, &sub_options));
		RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

        // TODO: service

       for (;;) {
         rclc_executor_spin_some(&executor, RCL_MS_TO_NS(9));  // 処理は1ms以内
         osDelay(1);  // FreeRTOSの1msスケジューリング
        }

       RCCHECK(rclc_executor_fini(&executor));
       RCCHECK(rcl_subscription_fini(&point_subscriber, &node));
       RCCHECK(rcl_subscription_fini(&subscriber, &node));
       RCCHECK(rcl_publisher_fini(&robstride_fb_publisher, &node));
       RCCHECK(rcl_node_fini(&node));
  }
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
  /* Infinite loop */
  for(;;)
  {
//	    		  Robstride_ControlEnable(&robstride_dev_info_global[i]);
//	    		  HAL_Delay(10);
//	    	  }
//
//	    	  Robstride_ChangeControl(&robstride_dev_info_global[0], ROBSTRIDE_CTRL_POS);
//	    	  HAL_Delay(10);
//
//	    	    float current_target;
//	    	    osMutexAcquire(robstridePosHandle, osWaitForever);
//	    	    current_target = robstride_target_value;
//	    	    osMutexRelease(robstridePosHandle);
//
//	    	  Robstride_SetTarget(&robstride_dev_info_global[0], current_target);
//	    	  printf("set target OK\n\r");
	  if(robstride_first_flag == true){
		   Robstride_SetTarget(&robstride_dev_info_global[0], robstride_target_value[0]);
		   Robstride_SetTarget(&robstride_dev_info_global[1], robstride_target_value[1]);
		   printf("target1:%f, target2:%f\n\r", robstride_target_value[0], robstride_target_value[1]);
	  }

      feedback_data[0] = Read_Robstride_FeedbackData(&robstride_dev_info_global[0]);
      feedback_data[1] = Read_Robstride_FeedbackData(&robstride_dev_info_global[1]);

      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
      osDelay(10);
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
  /* Infinite loop */
  for(;;)
  {
//	  RoboMas_SendRequest(robomas_dev_info_global, num_of_robomas, 1000.0f, &hcan1);//制御する
//	  RoboMas_SetTarget(&robomas_dev_info_global[0], 1.0f);//目標値を設定
//	  for(int i=0; i<num_of_robomas; i++){
//	      robomas_fb[i]=Get_RoboMas_FeedbackData(&robomas_dev_info_global[i]);
	      //フィードバックを受け取る
//	      printf("M3508%d : %d\r\n", i, (int)(robomas_fb[i].position));
	      //velocityとcurrentもある

    osDelay(1);
  }
  /* USER CODE END StartRobomasTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* Point.x/y をそれぞれ Robstride 02/05 の位置目標値として受け取る。 */
static void point_callback(const void *msgin)
{
  const geometry_msgs__msg__Point *point = msgin;

  robstride_target_value[0] = (float)point->x;
  robstride_target_value[1] = (float)point->y;
  robstride_first_flag = true;
}

/* Robstride 02 の位置・速度・電流を Point メッセージとして送信する。 */
static void feedback_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  (void)timer;
  (void)last_call_time;

  geometry_msgs__msg__Point feedback_msg = {
    .x = feedback_data[0].position,
    .y = feedback_data[0].velocity,
    .z = feedback_data[0].current,
  };
  RCCHECK(rcl_publish(&robstride_fb_publisher, &feedback_msg, NULL));
}

/* UInt8 トピックは現在は予約済み。受信処理を保持して将来の拡張に備える。 */
static void subscription_callback(const void *msgin)
{
  (void)msgin;
}

/* USER CODE END Application */


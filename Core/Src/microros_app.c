/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    microros_app.c
  * @brief   catch26_interface を使用した micro-ROS アプリケーション
  ******************************************************************************
  */
/* USER CODE END Header */

#include "microros_app.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"
#include "lwip/netif.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>
#include <uxr/client/transport.h>

#include "catch26_interface/msg/uros_f7_command.h"
#include "catch26_interface/msg/uros_f7_feedback.h"
#include "catch26_interface/msg/uros_f7_motor_unit_command.h"
#include "catch26_interface/msg/uros_f7_motor_unit_feedback.h"

#include "can_devices.h"
#include "CAN_RoboMas.h"
#include "CAN_Robstride.h"
#include "robstride_constant.h"
#include "RoboMas_utils.h"
#include "Robstride_utils.h"

#define MICROROS_AGENT_IP              "192.168.5.100"
#define MICROROS_COMMAND_TOPIC        "uros_f7_command"
#define MICROROS_FEEDBACK_TOPIC       "uros_f7_feedback"
#define MICROROS_MAX_MOTOR_UNITS      8U
#define MICROROS_FEEDBACK_PERIOD_MS   10U

/* custom transport / allocator は micro_ros_stm32cubemx_utils 側で実装する。 */
bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport,
                               const uint8_t *buf,
                               size_t len,
                               uint8_t *err);
size_t cubemx_transport_read(struct uxrCustomTransport *transport,
                              uint8_t *buf,
                              size_t len,
                              int timeout,
                              uint8_t *err);
void *microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void *microros_reallocate(void *pointer, size_t size, void *state);
void *microros_zero_allocate(size_t number_of_elements,
                             size_t size_of_element,
                             void *state);

#define RCCHECK(call) do { \
  rcl_ret_t rcl_ret = (call); \
  if (rcl_ret != RCL_RET_OK) { \
    printf("micro-ROS error: %d (line %d)\\r\\n", (int)rcl_ret, __LINE__); \
    rcl_reset_error(); \
  } \
} while (0)

static rcl_subscription_t command_subscriber;
static rcl_publisher_t feedback_publisher;
static catch26_interface__msg__UrosF7Command command_msg;
static catch26_interface__msg__UrosF7Command last_command_msg;
static catch26_interface__msg__UrosF7Feedback feedback_msg;

/* RobstrideTask が CAN の送信周期で参照する、最後に受信した指令。 */
static catch26_interface__msg__UrosF7MotorUnitCommand
    robstride_commands[ROBSTRIDE_DEVICE_STORAGE_COUNT];
static bool robstride_command_valid[ROBSTRIDE_DEVICE_STORAGE_COUNT];

static void command_callback(const void *msgin);
static void feedback_timer_callback(rcl_timer_t *timer, int64_t last_call_time);

/* CAN ライブラリが要求する void 戻り値の遅延関数へ CMSIS-RTOS を適合させる。 */
static void microros_delay(uint32_t delay_ms)
{
  (void)osDelay(delay_ms);
}

static bool initialize_command_message(
    catch26_interface__msg__UrosF7Command *msg)
{
  if (!catch26_interface__msg__UrosF7Command__init(msg)) {
    return false;
  }

  if (!catch26_interface__msg__UrosF7MotorUnitCommand__Sequence__init(
          &msg->command, MICROROS_MAX_MOTOR_UNITS)) {
    catch26_interface__msg__UrosF7Command__fini(msg);
    return false;
  }

  /* capacity は最大数、size は受信済み要素数として管理する。 */
  msg->command.size = 0U;
  return true;
}

static bool initialize_feedback_message(
    catch26_interface__msg__UrosF7Feedback *msg)
{
  if (!catch26_interface__msg__UrosF7Feedback__init(msg)) {
    return false;
  }

  if (!catch26_interface__msg__UrosF7MotorUnitFeedback__Sequence__init(
          &msg->feedback, MICROROS_MAX_MOTOR_UNITS)) {
    catch26_interface__msg__UrosF7Feedback__fini(msg);
    return false;
  }
  msg->feedback.size = 0U;

  /* UrosF7Feedback.last_command も bounded sequence なので受信可能数を確保する。 */
  if (!catch26_interface__msg__UrosF7MotorUnitCommand__Sequence__init(
          &msg->last_command.command, MICROROS_MAX_MOTOR_UNITS)) {
    catch26_interface__msg__UrosF7Feedback__fini(msg);
    return false;
  }
  msg->last_command.command.size = 0U;

  return true;
}

static void fini_messages(void)
{
  catch26_interface__msg__UrosF7Command__fini(&command_msg);
  catch26_interface__msg__UrosF7Command__fini(&last_command_msg);
  catch26_interface__msg__UrosF7Feedback__fini(&feedback_msg);
}

static bool device_info_matches(const catch26_interface__msg__DeviceInfo *info,
                                uint8_t type,
                                uint8_t id)
{
  return (info->type == type) && (info->id == id);
}

static int find_robstride_device(
    const catch26_interface__msg__DeviceInfo *info)
{
  for (uint32_t i = 0U; i < ROBSTRIDE_DEVICE_COUNT; ++i) {
    if (device_info_matches(info,
                            catch26_interface__msg__DeviceInfo__TYPE_ROBSTRIDE,
                            robstride_dev_info_global[i].device_id)) {
      return (int)i;
    }
  }

  return -1;
}

static int find_robomas_device(const catch26_interface__msg__DeviceInfo *info)
{
  for (uint32_t i = 0U; i < ROBOMAS_DEVICE_COUNT; ++i) {
    if (device_info_matches(info,
                            catch26_interface__msg__DeviceInfo__TYPE_ROBOMASTER,
                            robomas_dev_info_global[i].device_id)) {
      return (int)i;
    }
  }

  return -1;
}

static float command_target_value(
    const catch26_interface__msg__UrosF7MotorUnitCommand *command)
{
  switch (command->unit_option) {
    case catch26_interface__msg__UrosF7MotorUnitFeedback__STATE_POSITION:
      return command->position;
    case catch26_interface__msg__UrosF7MotorUnitFeedback__STATE_VELOCITY:
      return command->velocity;
    case catch26_interface__msg__UrosF7MotorUnitFeedback__STATE_CURRENT:
      return command->current;
    default:
      return 0.0f;
  }
}

static void apply_robstride_command(
    Robstride_DeviceInfo *device,
    const catch26_interface__msg__UrosF7MotorUnitCommand *command)
{
  const uint8_t option = command->unit_option;

  if (option == catch26_interface__msg__UrosF7MotorUnitFeedback__STATE_DISABLE) {
    Robstride_ControlDisable(device, microros_delay);
    return;
  }

  if (option < catch26_interface__msg__UrosF7MotorUnitFeedback__STATE_POSITION ||
      option > catch26_interface__msg__UrosF7MotorUnitFeedback__STATE_CURRENT) {
    return;
  }

  const ROBSTRIDE_CTRL_TYPE requested_mode = (ROBSTRIDE_CTRL_TYPE)option;
  if (device->ctrl_param.ctrl_type != requested_mode) {
    Robstride_SetControl(device, requested_mode, microros_delay);
  } else if (device->ctrl_param._enable_flag == 0U) {
    Robstride_ControlEnable(device, microros_delay);
  }

  Robstride_SetTarget(device, command_target_value(command));
}

static void apply_robomas_command(
    RoboMas_DeviceInfo *device,
    const catch26_interface__msg__UrosF7MotorUnitCommand *command)
{
  const uint8_t option = command->unit_option;

  if (option == catch26_interface__msg__UrosF7MotorUnitFeedback__STATE_DISABLE) {
    RoboMas_ControlDisable(device);
    return;
  }

  if (option < catch26_interface__msg__UrosF7MotorUnitFeedback__STATE_POSITION ||
      option > catch26_interface__msg__UrosF7MotorUnitFeedback__STATE_CURRENT) {
    return;
  }

  const ROBOMAS_CTRL_TYPE requested_mode = (ROBOMAS_CTRL_TYPE)(option - 1U);
  if (device->ctrl_param.ctrl_type != requested_mode) {
    RoboMas_ChangeControl(device, requested_mode);
  }
  RoboMas_ControlEnable(device);
  RoboMas_SetTarget(device, command_target_value(command));
}

static void remember_last_command(
    const catch26_interface__msg__UrosF7Command *command)
{
  const size_t count = (command->command.size < MICROROS_MAX_MOTOR_UNITS)
                           ? command->command.size
                           : MICROROS_MAX_MOTOR_UNITS;

  last_command_msg.option = command->option;
  last_command_msg.command.size = count;
  for (size_t i = 0U; i < count; ++i) {
    last_command_msg.command.data[i] = command->command.data[i];
  }
}

static void command_callback(const void *msgin)
{
  const catch26_interface__msg__UrosF7Command *command = msgin;
  const size_t command_count = (command->command.size < MICROROS_MAX_MOTOR_UNITS)
                                 ? command->command.size
                                 : MICROROS_MAX_MOTOR_UNITS;

  remember_last_command(command);

  for (size_t i = 0U; i < command_count; ++i) {
    const catch26_interface__msg__UrosF7MotorUnitCommand *unit =
        &command->command.data[i];
    const int robstride_index = find_robstride_device(&unit->info);
    const int robomas_index = find_robomas_device(&unit->info);

    if (robstride_index >= 0) {
      robstride_commands[robstride_index] = *unit;
      robstride_command_valid[robstride_index] = true;
      apply_robstride_command(&robstride_dev_info_global[robstride_index], unit);
    } else if (robomas_index >= 0) {
      apply_robomas_command(&robomas_dev_info_global[robomas_index], unit);
    }
  }
}

void MicroRos_RefreshRobstrideTargets(void)
{
  for (uint32_t i = 0U; i < ROBSTRIDE_DEVICE_COUNT; ++i) {
    if (robstride_command_valid[i]) {
      const catch26_interface__msg__UrosF7MotorUnitCommand *command =
          &robstride_commands[i];
      if (command->unit_option !=
              catch26_interface__msg__UrosF7MotorUnitFeedback__STATE_DISABLE &&
          command->unit_option >=
              catch26_interface__msg__UrosF7MotorUnitFeedback__STATE_POSITION &&
          command->unit_option <=
              catch26_interface__msg__UrosF7MotorUnitFeedback__STATE_CURRENT) {
        Robstride_SetTarget(&robstride_dev_info_global[i],
                            command_target_value(command));
      }
    }
  }
}

static void fill_local_time(builtin_interfaces__msg__Time *time_msg)
{
  int64_t now_ns;

  if (rmw_uros_epoch_synchronized()) {
    now_ns = rmw_uros_epoch_nanos();
  } else {
    /* Agent と時刻同期できていない場合でも、ローカル時刻を返す。 */
    now_ns = (int64_t)HAL_GetTick() * 1000000LL;
  }

  time_msg->sec = (int32_t)(now_ns / 1000000000LL);
  time_msg->nanosec = (uint32_t)(now_ns % 1000000000LL);
}

static uint8_t robstride_state(const Robstride_FeedbackData *feedback)
{
  if (feedback->mode_status != ROBSTRIDE_STATE_ENABLE) {
    return catch26_interface__msg__UrosF7MotorUnitFeedback__STATE_DISABLE;
  }

  if (feedback->run_mode >= ROBSTRIDE_CTRL_POS &&
      feedback->run_mode <= ROBSTRIDE_CTRL_CURRENT) {
    return feedback->run_mode;
  }

  return catch26_interface__msg__UrosF7MotorUnitFeedback__STATE_DISABLE;
}

static void set_robstride_feedback(
    catch26_interface__msg__UrosF7MotorUnitFeedback *output,
    uint32_t index)
{
  const Robstride_DeviceInfo *device = &robstride_dev_info_global[index];
  const Robstride_FeedbackData *feedback = &feedback_data[index];

  output->info.type = catch26_interface__msg__DeviceInfo__TYPE_ROBSTRIDE;
  output->info.id = device->device_id;
  output->position = feedback->position;
  output->velocity = feedback->velocity;
  output->current = feedback->current;
  output->state = robstride_state(feedback);
  output->unit_message_code = feedback->get_flag
                                  ? catch26_interface__msg__UrosF7MotorUnitFeedback__CODE_NORMAL
                                  : catch26_interface__msg__UrosF7MotorUnitFeedback__CODE_DISCONNECTION;
}

static void set_robomas_feedback(
    catch26_interface__msg__UrosF7MotorUnitFeedback *output,
    uint32_t index)
{
  const RoboMas_DeviceInfo *device = &robomas_dev_info_global[index];
  const RoboMas_FeedbackData *feedback = &robomas_fb[index];

  output->info.type = catch26_interface__msg__DeviceInfo__TYPE_ROBOMASTER;
  output->info.id = device->device_id;
  output->position = feedback->position;
  output->velocity = feedback->velocity;
  output->current = feedback->current;
  output->state = device->ctrl_param._enable_flag
                    ? (uint8_t)(device->ctrl_param.ctrl_type + 1U)
                    : catch26_interface__msg__UrosF7MotorUnitFeedback__STATE_DISABLE;
  output->unit_message_code = feedback->get_flag
                                ? catch26_interface__msg__UrosF7MotorUnitFeedback__CODE_NORMAL
                                : catch26_interface__msg__UrosF7MotorUnitFeedback__CODE_DISCONNECTION;
}

static void feedback_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  (void)timer;
  (void)last_call_time;

  size_t feedback_count = 0U;
  bool all_connected = true;

  fill_local_time(&feedback_msg.local_time);
  feedback_msg.last_command.option = last_command_msg.option;
  feedback_msg.last_command.command.size = last_command_msg.command.size;
  for (size_t i = 0U; i < last_command_msg.command.size; ++i) {
    feedback_msg.last_command.command.data[i] = last_command_msg.command.data[i];
  }

  for (uint32_t i = 0U; i < ROBSTRIDE_DEVICE_COUNT; ++i) {
    if (feedback_count >= MICROROS_MAX_MOTOR_UNITS) {
      break;
    }
    set_robstride_feedback(&feedback_msg.feedback.data[feedback_count], i);
    all_connected = all_connected && (feedback_data[i].get_flag != 0U);
    ++feedback_count;
  }

  for (uint32_t i = 0U; i < ROBOMAS_DEVICE_COUNT; ++i) {
    if (feedback_count >= MICROROS_MAX_MOTOR_UNITS) {
      break;
    }
    set_robomas_feedback(&feedback_msg.feedback.data[feedback_count], i);
    all_connected = all_connected && (robomas_fb[i].get_flag != 0U);
    ++feedback_count;
  }

  feedback_msg.feedback.size = feedback_count;
  (void)rosidl_runtime_c__String__assign(
      &feedback_msg.message, all_connected ? "ok" : "motor disconnected");

  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
  rcl_ret_t ret = rcl_publish(&feedback_publisher, &feedback_msg, NULL);
  if (ret != RCL_RET_OK) {
    printf("micro-ROS feedback publish error: %d\r\n", (int)ret);
    rcl_reset_error();
  }
}

static void wait_for_network(void)
{
  extern struct netif gnetif;
  struct netif *netif_ptr = netif_default;

  while (netif_ptr == NULL || netif_ptr->ip_addr.addr == 0U) {
    osDelay(100U);
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
  }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

  while (!netif_is_link_up(&gnetif)) {
    osDelay(1000U);
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
  }
}

static bool initialize_messages(void)
{
  command_msg = (catch26_interface__msg__UrosF7Command){0};
  last_command_msg = (catch26_interface__msg__UrosF7Command){0};
  feedback_msg = (catch26_interface__msg__UrosF7Feedback){0};

  if (!initialize_command_message(&command_msg)) {
    return false;
  }
  if (!initialize_command_message(&last_command_msg)) {
    catch26_interface__msg__UrosF7Command__fini(&command_msg);
    return false;
  }
  if (!initialize_feedback_message(&feedback_msg)) {
    catch26_interface__msg__UrosF7Command__fini(&command_msg);
    catch26_interface__msg__UrosF7Command__fini(&last_command_msg);
    return false;
  }

  return true;
}

static void reset_command_state(void)
{
  for (uint32_t i = 0U; i < ROBSTRIDE_DEVICE_STORAGE_COUNT; ++i) {
    robstride_command_valid[i] = false;
    robstride_commands[i] = (catch26_interface__msg__UrosF7MotorUnitCommand){0};
  }
}

void MicroRosTask_Run(void)
{
  printf("Start Micro-ROS Task\r\n");
  wait_for_network();

  for (;;) {
    rmw_uros_set_custom_transport(
        false,
        (void *)MICROROS_AGENT_IP,
        cubemx_transport_open,
        cubemx_transport_close,
        cubemx_transport_write,
        cubemx_transport_read);

    while (rmw_uros_ping_agent(1000, 5) != RMW_RET_OK) {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
      osDelay(100U);
    }
    printf("micro-ROS agent connected\r\n");
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate = microros_zero_allocate;
    if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
      printf("micro-ROS allocator initialization failed\r\n");
      osDelay(1000U);
      continue;
    }

    if (!initialize_messages()) {
      printf("micro-ROS message initialization failed\r\n");
      osDelay(1000U);
      continue;
    }
    reset_command_state();

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rclc_support_t support = {0};
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rcl_node_t node = rcl_get_zero_initialized_node();
    rcl_node_options_t node_options = rcl_node_get_default_options();
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    rcl_timer_t feedback_timer = rcl_get_zero_initialized_timer();

    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, 30));
    RCCHECK(rclc_support_init_with_options(
        &support, 0, NULL, &init_options, &allocator));
    RCCHECK(rclc_node_init_with_options(
        &node, "f7_mros_node", "", &support, &node_options));
    RCCHECK(rclc_executor_init(&executor, &support.context, 2U, &allocator));

    rmw_qos_profile_t command_qos = rmw_qos_profile_default;
    command_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    RCCHECK(rclc_subscription_init(
        &command_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(catch26_interface, msg, UrosF7Command),
        MICROROS_COMMAND_TOPIC,
        &command_qos));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &command_subscriber, &command_msg, &command_callback, ON_NEW_DATA));

    rmw_qos_profile_t feedback_qos = rmw_qos_profile_default;
    feedback_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    RCCHECK(rclc_publisher_init(
        &feedback_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(catch26_interface, msg, UrosF7Feedback),
        MICROROS_FEEDBACK_TOPIC,
        &feedback_qos));
    RCCHECK(rclc_timer_init_default(
        &feedback_timer,
        &support,
        RCL_MS_TO_NS(MICROROS_FEEDBACK_PERIOD_MS),
        &feedback_timer_callback));
    RCCHECK(rclc_executor_add_timer(&executor, &feedback_timer));
    printf("micro-ROS initialized\r\n");

    for (;;) {
      const rcl_ret_t spin_result =
          rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
      if (spin_result != RCL_RET_OK && spin_result != RCL_RET_TIMEOUT) {
        printf("micro-ROS executor stopped: %d\r\n", (int)spin_result);
        rcl_reset_error();
        break;
      }
      osDelay(1U);
    }

    RCCHECK(rclc_executor_fini(&executor));
    RCCHECK(rcl_timer_fini(&feedback_timer));
    RCCHECK(rcl_subscription_fini(&command_subscriber, &node));
    RCCHECK(rcl_publisher_fini(&feedback_publisher, &node));
    RCCHECK(rcl_node_fini(&node));
    RCCHECK(rclc_support_fini(&support));
    RCCHECK(rcl_init_options_fini(&init_options));
    fini_messages();
    (void)cubemx_transport_close(NULL);
    osDelay(1000U);
  }
}

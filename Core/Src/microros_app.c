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
#include <math.h>
#include <string.h>

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
#include "catch26_interface/srv/uros_f7_param.h"

#include "can_devices.h"
#include "CAN_RoboMas.h"
#include "CAN_Robstride.h"
#include "robstride_constant.h"
#include "RoboMas_utils.h"
#include "Robstride_utils.h"

#define MICROROS_AGENT_IP              "192.168.5.100"
#define MICROROS_COMMAND_TOPIC        "uros_f7_command"
#define MICROROS_FEEDBACK_TOPIC       "uros_f7_feedback"
#define MICROROS_PARAMETER_SERVICE    "uros_f7_param"
#define MICROROS_MAX_MOTOR_UNITS      8U
#define MICROROS_RESPONSE_MESSAGE_MAX 32U
#define MICROROS_TARGET_STORAGE_SIZE  17U
#define MICROROS_COMMAND_STORAGE_SIZE 17U
#define MICROROS_RESPONSE_STORAGE_SIZE 33U
#define MICROROS_FEEDBACK_PERIOD_MS   10U
#define MICROROS_COMMAND_NOMINAL_HZ   500U
#define MICROROS_DIAGNOSTIC_PERIOD_MS 1000U

/* UrosF7Param.mode の共通値。3 は将来の自作位置制御用に予約する。 */
#define MICROROS_MODE_POSITION        0U
#define MICROROS_MODE_VELOCITY        1U
#define MICROROS_MODE_CURRENT         2U
#define MICROROS_MODE_CUSTOM_POSITION 3U

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
static rcl_service_t parameter_service;
static catch26_interface__msg__UrosF7Command command_msg;
static catch26_interface__msg__UrosF7Command last_command_msg;
static catch26_interface__msg__UrosF7Feedback feedback_msg;
static catch26_interface__srv__UrosF7Param_Request parameter_request;
static catch26_interface__srv__UrosF7Param_Response parameter_response;
/* micro-ROSのbounded string受信器が参照する固定バッファ。 */
static char parameter_target_storage[MICROROS_TARGET_STORAGE_SIZE];
static char parameter_command_storage[MICROROS_COMMAND_STORAGE_SIZE];
static char parameter_response_storage[MICROROS_RESPONSE_STORAGE_SIZE];

/* RobstrideTask が CAN の送信周期で参照する、最後に受信した指令。 */
static catch26_interface__msg__UrosF7MotorUnitCommand
    robstride_commands[ROBSTRIDE_DEVICE_STORAGE_COUNT];
static volatile bool robstride_command_valid[ROBSTRIDE_DEVICE_STORAGE_COUNT];
static volatile bool robstride_command_pending[ROBSTRIDE_DEVICE_STORAGE_COUNT];

/* 受信コールバックでは数えるだけにし、UART出力は低頻度のタスク側で行う。 */
static volatile uint32_t microros_command_received_count = 0U;
static volatile uint32_t microros_command_coalesced_count = 0U;
static uint32_t microros_diagnostics_last_tick = 0U;
static bool microros_diagnostics_started = false;
static uint32_t microros_last_feedback_error_tick = 0U;

static void command_callback(const void *msgin);
static void parameter_service_callback(const void *request_msg,
                                       void *response_msg);
static void feedback_timer_callback(rcl_timer_t *timer, int64_t last_call_time);

static uint32_t take_counter(volatile uint32_t *counter)
{
  const uint32_t primask = __get_PRIMASK();
  uint32_t value;

  __disable_irq();
  value = *counter;
  *counter = 0U;
  __set_PRIMASK(primask);
  return value;
}
typedef enum {
  MICROROS_TARGET_ROBSTRIDE = 0,
  MICROROS_TARGET_ROBOMASTER = 1
} microros_target_type_t;

static char ascii_lower(char value)
{
  if (value >= 'A' && value <= 'Z') {
    return (char)(value + ('a' - 'A'));
  }
  return value;
}

static bool ros_string_equals_literal(const rosidl_runtime_c__String *value,
                                      const char *literal)
{
  size_t literal_size = 0U;

  if (value == NULL || value->data == NULL || literal == NULL) {
    return false;
  }
  while (literal[literal_size] != '\0') {
    ++literal_size;
  }
  if (value->size != literal_size) {
    return false;
  }
  for (size_t i = 0U; i < literal_size; ++i) {
    if (ascii_lower(value->data[i]) != ascii_lower(literal[i])) {
      return false;
    }
  }
  return true;
}

static bool ros_string_starts_with_literal(const rosidl_runtime_c__String *value,
                                           const char *literal,
                                           size_t *literal_size_out)
{
  size_t literal_size = 0U;

  if (value == NULL || value->data == NULL || literal == NULL) {
    return false;
  }
  while (literal[literal_size] != '\0') {
    ++literal_size;
  }
  if (value->size < literal_size) {
    return false;
  }
  for (size_t i = 0U; i < literal_size; ++i) {
    if (ascii_lower(value->data[i]) != ascii_lower(literal[i])) {
      return false;
    }
  }
  if (literal_size_out != NULL) {
    *literal_size_out = literal_size;
  }
  return true;
}

/* target は "robstride:<CAN ID>" または "robomaster:<CAN ID>" とする。 */
static bool parse_motor_target(const rosidl_runtime_c__String *target,
                               microros_target_type_t *target_type,
                               uint8_t *device_id)
{
  size_t prefix_size = 0U;
  microros_target_type_t parsed_type;

  if (ros_string_starts_with_literal(target, "robstride:", &prefix_size)) {
    parsed_type = MICROROS_TARGET_ROBSTRIDE;
  } else if (ros_string_starts_with_literal(
                 target, "robomaster:", &prefix_size)) {
    parsed_type = MICROROS_TARGET_ROBOMASTER;
  } else {
    return false;
  }

  if (target->size <= prefix_size) {
    return false;
  }

  uint32_t parsed_id = 0U;
  for (size_t i = prefix_size; i < target->size; ++i) {
    const char digit = target->data[i];
    if (digit < '0' || digit > '9') {
      return false;
    }
    parsed_id = parsed_id * 10U + (uint32_t)(digit - '0');
    if (parsed_id > 255U) {
      return false;
    }
  }

  if (parsed_id == 0U) {
    return false;
  }
  *target_type = parsed_type;
  *device_id = (uint8_t)parsed_id;
  return true;
}

static int find_robstride_device_by_id(uint8_t device_id)
{
  for (uint32_t i = 0U; i < ROBSTRIDE_DEVICE_COUNT; ++i) {
    if (robstride_dev_info_global[i].device_id == device_id) {
      return (int)i;
    }
  }
  return -1;
}

static int find_robomas_device_by_id(uint8_t device_id)
{
  for (uint32_t i = 0U; i < ROBOMAS_DEVICE_COUNT; ++i) {
    if (robomas_dev_info_global[i].device_id == device_id) {
      return (int)i;
    }
  }
  return -1;
}

static bool parse_mode_data(float data, uint8_t *mode)
{
  if (!isfinite(data) || data < 0.0f || data > (float)MICROROS_MODE_CUSTOM_POSITION) {
    return false;
  }

  const uint8_t parsed_mode = (uint8_t)data;
  if ((float)parsed_mode != data) {
    return false;
  }
  *mode = parsed_mode;
  return true;
}

static bool map_robstride_mode(uint8_t mode, ROBSTRIDE_CTRL_TYPE *ctrl_type)
{
  switch (mode) {
    case MICROROS_MODE_POSITION:
      *ctrl_type = ROBSTRIDE_CTRL_POS;
      return true;
    case MICROROS_MODE_VELOCITY:
      *ctrl_type = ROBSTRIDE_CTRL_VEL;
      return true;
    case MICROROS_MODE_CURRENT:
      *ctrl_type = ROBSTRIDE_CTRL_CURRENT;
      return true;
    default:
      return false;
  }
}

static bool map_robomas_mode(uint8_t mode, ROBOMAS_CTRL_TYPE *ctrl_type)
{
  switch (mode) {
    case MICROROS_MODE_POSITION:
      *ctrl_type = ROBOMAS_CTRL_POS;
      return true;
    case MICROROS_MODE_VELOCITY:
      *ctrl_type = ROBOMAS_CTRL_VEL;
      return true;
    case MICROROS_MODE_CURRENT:
      *ctrl_type = ROBOMAS_CTRL_CURRENT;
      return true;
    default:
      return false;
  }
}

static void microros_delay(uint32_t milliseconds)
{
  /* サービス処理中もRobstrideTaskへCPUを返す。 */
  (void)osDelay(milliseconds);
}

static void set_parameter_response(
    catch26_interface__srv__UrosF7Param_Response *response,
    bool success,
    const char *message)
{
  size_t message_size = 0U;

  if (message != NULL) {
    while (message[message_size] != '\0' &&
           message_size < MICROROS_RESPONSE_MESSAGE_MAX) {
      ++message_size;
    }
  }

  response->success = success;
  /* response側も固定領域へ直接書き込み、bounded string上限を保証する。 */
  response->message.data = parameter_response_storage;
  response->message.capacity = sizeof(parameter_response_storage);
  if (message_size > 0U && message != NULL) {
    memcpy(response->message.data, message, message_size);
  }
  response->message.data[message_size] = '\0';
  response->message.size = message_size;
}

static void invalidate_robstride_command(Robstride_DeviceInfo *device)
{
  const uint32_t index = (uint32_t)(device - robstride_dev_info_global);
  if (index >= ROBSTRIDE_DEVICE_COUNT) {
    return;
  }

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  robstride_command_valid[index] = false;
  robstride_command_pending[index] = false;
  __set_PRIMASK(primask);
}

static bool set_robstride_mode(Robstride_DeviceInfo *device, uint8_t mode)
{
  ROBSTRIDE_CTRL_TYPE ctrl_type;
  if (!map_robstride_mode(mode, &ctrl_type)) {
    return false;
  }

  /* モード遷移中に旧ROS目標を低優先度CANへ再投入させない。 */
  invalidate_robstride_command(device);
  return Robstride_ServiceChangeControl(device, ctrl_type, microros_delay) != 0U;
}

static bool set_robomas_mode(RoboMas_DeviceInfo *device, uint8_t mode)
{
  ROBOMAS_CTRL_TYPE ctrl_type;
  if (!map_robomas_mode(mode, &ctrl_type)) {
    return false;
  }

  const bool was_enabled = device->ctrl_param._enable_flag;
  RoboMas_ChangeControl(device, ctrl_type);
  if (was_enabled) {
    RoboMas_ControlEnable(device);
  }
  return true;
}

static void parameter_service_callback(const void *request_msg,
                                       void *response_msg)
{
  const catch26_interface__srv__UrosF7Param_Request *request = request_msg;
  catch26_interface__srv__UrosF7Param_Response *response = response_msg;
  microros_target_type_t target_type;
  uint8_t device_id;

  if (response == NULL) {
    return;
  }
  set_parameter_response(response, false, "invalid request");
  if (request == NULL) {
    return;
  }

  if (!parse_motor_target(&request->target, &target_type, &device_id)) {
    set_parameter_response(response, false, "invalid target");
    return;
  }

  const bool is_mode = ros_string_equals_literal(&request->command, "mode");
  const bool is_enable = ros_string_equals_literal(&request->command, "enable");
  const bool is_disable = ros_string_equals_literal(&request->command, "disable");
  if (!is_mode && !is_enable && !is_disable) {
    set_parameter_response(response, false, "invalid command");
    return;
  }

  int device_index;
  if (target_type == MICROROS_TARGET_ROBSTRIDE) {
    device_index = find_robstride_device_by_id(device_id);
  } else {
    device_index = find_robomas_device_by_id(device_id);
  }
  if (device_index < 0) {
    set_parameter_response(response, false, "target motor is not configured");
    return;
  }

  if (is_mode) {
    uint8_t mode;
    if (!parse_mode_data(request->data, &mode)) {
      set_parameter_response(response, false, "mode data must be integer 0..3");
      return;
    }
    if (mode == MICROROS_MODE_CUSTOM_POSITION) {
      set_parameter_response(response, false, "mode 3 is reserved");
      return;
    }

    const bool changed = (target_type == MICROROS_TARGET_ROBSTRIDE)
                           ? set_robstride_mode(
                               &robstride_dev_info_global[device_index], mode)
                           : set_robomas_mode(
                               &robomas_dev_info_global[device_index], mode);
    if (changed) {
      set_parameter_response(response, true, "mode changed");
    } else {
      set_parameter_response(response, false, "mode change failed");
    }
    return;
  }

  /* Enable/Disable は data を参照しない。 */
  if (target_type == MICROROS_TARGET_ROBSTRIDE) {
    Robstride_DeviceInfo *device = &robstride_dev_info_global[device_index];
    invalidate_robstride_command(device);
    if (Read_Robstride_FeedbackData(device).get_flag == 0U) {
      set_parameter_response(response, false, "motor disconnected");
      return;
    }
    const uint8_t control_ok = is_enable
                                 ? Robstride_ControlEnable(device, microros_delay)
                                 : Robstride_ControlDisable(device, microros_delay);
    if (!control_ok) {
      set_parameter_response(response, false,
                             is_enable ? "enable timeout" : "disable timeout");
      return;
    }
  } else {
    RoboMas_DeviceInfo *device = &robomas_dev_info_global[device_index];
    if (is_enable) {
      RoboMas_ControlEnable(device);
    } else {
      RoboMas_ControlDisable(device);
    }
  }

  set_parameter_response(response, true, is_enable ? "enabled" : "disabled");
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

static bool initialize_parameter_messages(void)
{
  parameter_request =
      (catch26_interface__srv__UrosF7Param_Request){0};
  parameter_response =
      (catch26_interface__srv__UrosF7Param_Response){0};

  if (!catch26_interface__srv__UrosF7Param_Request__init(&parameter_request)) {
    return false;
  }
  if (!catch26_interface__srv__UrosF7Param_Response__init(&parameter_response)) {
    catch26_interface__srv__UrosF7Param_Request__fini(&parameter_request);
    return false;
  }

  /* bounded stringのmicro-ROS deserializerへ容量を渡す。 */
  parameter_request.target.data = parameter_target_storage;
  parameter_request.target.size = 0U;
  parameter_request.target.capacity = sizeof(parameter_target_storage);
  parameter_request.target.data[0] = '\0';
  parameter_request.command.data = parameter_command_storage;
  parameter_request.command.size = 0U;
  parameter_request.command.capacity = sizeof(parameter_command_storage);
  parameter_request.command.data[0] = '\0';
  parameter_response.message.data = parameter_response_storage;
  parameter_response.message.size = 0U;
  parameter_response.message.capacity = sizeof(parameter_response_storage);
  parameter_response.message.data[0] = '\0';
  return true;
}

static void fini_messages(void)
{
  catch26_interface__msg__UrosF7Command__fini(&command_msg);
  catch26_interface__msg__UrosF7Command__fini(&last_command_msg);
  catch26_interface__msg__UrosF7Feedback__fini(&feedback_msg);
  /* 固定バッファをrosidl allocatorが解放しないように切り離す。 */
  parameter_request.target.data = NULL;
  parameter_request.target.size = 0U;
  parameter_request.target.capacity = 0U;
  parameter_request.command.data = NULL;
  parameter_request.command.size = 0U;
  parameter_request.command.capacity = 0U;
  parameter_response.message.data = NULL;
  parameter_response.message.size = 0U;
  parameter_response.message.capacity = 0U;
  catch26_interface__srv__UrosF7Param_Request__fini(&parameter_request);
  catch26_interface__srv__UrosF7Param_Response__fini(&parameter_response);
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

/* unit_option は将来の拡張用であり、現時点では制御モードに使用しない。 */
static float robstride_command_target_value(
    const Robstride_DeviceInfo *device,
    const catch26_interface__msg__UrosF7MotorUnitCommand *command)
{
  switch (device->ctrl_param.ctrl_type) {
    case ROBSTRIDE_CTRL_POS:
      return command->position;
    case ROBSTRIDE_CTRL_VEL:
      return command->velocity;
    case ROBSTRIDE_CTRL_CURRENT:
      return command->current;
    default:
      return 0.0f;
  }
}

static float robomas_command_target_value(
    const RoboMas_DeviceInfo *device,
    const catch26_interface__msg__UrosF7MotorUnitCommand *command)
{
  switch (device->ctrl_param.ctrl_type) {
    case ROBOMAS_CTRL_POS:
      return command->position;
    case ROBOMAS_CTRL_VEL:
      return command->velocity;
    case ROBOMAS_CTRL_CURRENT:
      return command->current;
    default:
      return 0.0f;
  }
}

static void apply_robstride_command(
    Robstride_DeviceInfo *device,
    const catch26_interface__msg__UrosF7MotorUnitCommand *command)
{
  /* Robstride_SetTarget() 内で位置指令から offset_pos を減算する。 */
  Robstride_SetTarget(device, robstride_command_target_value(device, command));
}

static void apply_robomas_command(
    RoboMas_DeviceInfo *device,
    const catch26_interface__msg__UrosF7MotorUnitCommand *command)
{
  RoboMas_SetTarget(device, robomas_command_target_value(device, command));
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

  {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    ++microros_command_received_count;
    __set_PRIMASK(primask);
  }

  remember_last_command(command);

  for (size_t i = 0U; i < command_count; ++i) {
    const catch26_interface__msg__UrosF7MotorUnitCommand *unit =
        &command->command.data[i];
    const int robstride_index = find_robstride_device(&unit->info);
    const int robomas_index = find_robomas_device(&unit->info);

    if (robstride_index >= 0) {
      const uint32_t primask = __get_PRIMASK();

      /*
       * 受信側とRobstrideTaskは別タスクなので、構造体のコピーとpending
       * 更新を短いクリティカルセクションにまとめる。pending中の古い値は
       * 破棄し、常に最新値だけを制御タスクへ渡す。
       */
      __disable_irq();
      if (robstride_command_pending[robstride_index]) {
        ++microros_command_coalesced_count;
      }
      robstride_commands[robstride_index] = *unit;
      robstride_command_valid[robstride_index] = true;
      robstride_command_pending[robstride_index] = true;
      __set_PRIMASK(primask);
    } else if (robomas_index >= 0) {
      apply_robomas_command(&robomas_dev_info_global[robomas_index], unit);
    }
  }
}

void MicroRos_ApplyPendingRobstrideCommands(void)
{
  for (uint32_t i = 0U; i < ROBSTRIDE_DEVICE_COUNT; ++i) {
    catch26_interface__msg__UrosF7MotorUnitCommand command = {0};
    bool pending;
    const uint32_t primask = __get_PRIMASK();

    /* pendingを先に消費してからCAN処理を行う。処理中の新着値は次周回へ残る。 */
    __disable_irq();
    pending = robstride_command_pending[i];
    if (pending) {
      command = robstride_commands[i];
      robstride_command_pending[i] = false;
    }
    __set_PRIMASK(primask);

    if (pending) {
      apply_robstride_command(&robstride_dev_info_global[i],
                              &command);
    }
  }
}

void MicroRos_RefreshRobstrideTargets(void)
{
  for (uint32_t i = 0U; i < ROBSTRIDE_DEVICE_COUNT; ++i) {
    catch26_interface__msg__UrosF7MotorUnitCommand command = {0};
    bool valid;
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    valid = robstride_command_valid[i];
    if (valid) {
      command = robstride_commands[i];
    }
    __set_PRIMASK(primask);

    if (valid) {
      Robstride_SetTarget(&robstride_dev_info_global[i],
                          robstride_command_target_value(
                              &robstride_dev_info_global[i], &command));
    }
  }
}

void MicroRos_ReportDiagnostics(void)
{
  const uint32_t now = HAL_GetTick();

  if (!microros_diagnostics_started) {
    microros_diagnostics_started = true;
    microros_diagnostics_last_tick = now;
    return;
  }

  if ((uint32_t)(now - microros_diagnostics_last_tick) <
      MICROROS_DIAGNOSTIC_PERIOD_MS) {
    return;
  }
  microros_diagnostics_last_tick = now;

  const uint32_t received = take_counter(&microros_command_received_count);
  const uint32_t coalesced = take_counter(&microros_command_coalesced_count);
  const uint32_t ring_overrun = Robstride_TakeTxRingOverrunCount();
  const uint32_t priority_queue_full = Robstride_TakePriorityQueueFullCount();
  const uint32_t tx_errors = Robstride_TakeTxErrorCount();
  const uint32_t can_errors = Robstride_TakeCanErrorCount();
  const uint32_t can_error_code = Robstride_TakeCanErrorCode();

  if (received > MICROROS_COMMAND_NOMINAL_HZ || coalesced > 0U) {
    printf("Warning: uros_f7_command overload: %lu callbacks/s, "
           "coalesced=%lu; latest-value control continues\r\n",
           (unsigned long)received,
           (unsigned long)coalesced);
  }

  if (ring_overrun > 0U || priority_queue_full > 0U || tx_errors > 0U ||
      can_errors > 0U ||
      can_error_code != HAL_CAN_ERROR_NONE) {
    printf("Warning: Robstride CAN congestion: ring_overrun=%lu, "
           "priority_queue_full=%lu, tx_errors=%lu, can_events=%lu, "
           "code=0x%08lx; continuing\r\n",
           (unsigned long)ring_overrun,
           (unsigned long)priority_queue_full,
           (unsigned long)tx_errors,
           (unsigned long)can_errors,
           (unsigned long)can_error_code);
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

/* 直近に設定した制御モードを返す。無効化後も設定済みのモードは保持する。 */
static uint8_t robstride_state(
    const Robstride_DeviceInfo *device,
    const Robstride_FeedbackData *feedback)
{
  if (device->ctrl_param._enable_flag == 0U ||
      feedback->mode_status != ROBSTRIDE_STATE_ENABLE) {
    return catch26_interface__msg__UrosF7MotorUnitFeedback__STATE_DISABLE;
  }

  if (device->ctrl_param.ctrl_type >= ROBSTRIDE_CTRL_POS &&
      device->ctrl_param.ctrl_type <= ROBSTRIDE_CTRL_CURRENT) {
    return (uint8_t)device->ctrl_param.ctrl_type;
  }

  return catch26_interface__msg__UrosF7MotorUnitFeedback__STATE_DISABLE;
}

static void set_robstride_feedback(
    catch26_interface__msg__UrosF7MotorUnitFeedback *output,
    uint32_t index)
{
  const Robstride_DeviceInfo *device = &robstride_dev_info_global[index];
  const Robstride_FeedbackData feedback =
      CanDevices_GetRobstrideFeedback(index);

  output->info.type = catch26_interface__msg__DeviceInfo__TYPE_ROBSTRIDE;
  output->info.id = device->device_id;
  output->position = feedback.position;
  output->velocity = feedback.velocity;
  output->current = feedback.current;
  output->state = robstride_state(device, &feedback);
  output->unit_message_code = feedback.get_flag
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
    all_connected = all_connected &&
                    (CanDevices_GetRobstrideFeedback(i).get_flag != 0U);
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
    const uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - microros_last_feedback_error_tick) >=
        MICROROS_DIAGNOSTIC_PERIOD_MS) {
      printf("micro-ROS feedback publish error: %d\r\n", (int)ret);
      microros_last_feedback_error_tick = now;
    }
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
  printf("F7 Ethernet IP configured\r\n");
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

  while (!netif_is_link_up(&gnetif)) {
    osDelay(1000U);
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
  }
  printf("F7 Ethernet link up\r\n");
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
  if (!initialize_parameter_messages()) {
    catch26_interface__msg__UrosF7Command__fini(&command_msg);
    catch26_interface__msg__UrosF7Command__fini(&last_command_msg);
    catch26_interface__msg__UrosF7Feedback__fini(&feedback_msg);
    return false;
  }

  return true;
}

static void reset_command_state(void)
{
  const uint32_t primask = __get_PRIMASK();

  __disable_irq();
  for (uint32_t i = 0U; i < ROBSTRIDE_DEVICE_STORAGE_COUNT; ++i) {
    robstride_command_valid[i] = false;
    robstride_command_pending[i] = false;
    robstride_commands[i] = (catch26_interface__msg__UrosF7MotorUnitCommand){0};
  }
  microros_command_received_count = 0U;
  microros_command_coalesced_count = 0U;
  __set_PRIMASK(primask);
}

void MicroRosTask_Run(void)
{
  printf("Start Micro-ROS Task\r\n");
  wait_for_network();
  printf("micro-ROS waiting for agent at %s:8888\r\n", MICROROS_AGENT_IP);

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
    RCCHECK(rclc_executor_init(&executor, &support.context, 3U, &allocator));
    /* rcl_waitで受信待ちし続けず、制御タスクへCPUを返す。 */
    RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(0)));

    rmw_qos_profile_t command_qos = rmw_qos_profile_default;
    command_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    command_qos.depth = 1U;
    command_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    command_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    RCCHECK(rclc_subscription_init(
        &command_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(catch26_interface, msg, UrosF7Command),
        MICROROS_COMMAND_TOPIC,
        &command_qos));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &command_subscriber, &command_msg, &command_callback, ON_NEW_DATA));

    RCCHECK(rclc_service_init_default(
        &parameter_service,
        &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(catch26_interface, srv, UrosF7Param),
        MICROROS_PARAMETER_SERVICE));
    RCCHECK(rclc_executor_add_service(
        &executor,
        &parameter_service,
        &parameter_request,
        &parameter_response,
        &parameter_service_callback));

    rmw_qos_profile_t feedback_qos = rmw_qos_profile_default;
    feedback_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    feedback_qos.depth = 1U;
    feedback_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    feedback_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
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
          rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));
      if (spin_result != RCL_RET_OK && spin_result != RCL_RET_TIMEOUT) {
        printf("micro-ROS executor stopped: %d\r\n", (int)spin_result);
        rcl_reset_error();
        break;
      }
      osDelay(1U);
    }

    RCCHECK(rclc_executor_fini(&executor));
    RCCHECK(rcl_timer_fini(&feedback_timer));
    RCCHECK(rcl_service_fini(&parameter_service, &node));
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

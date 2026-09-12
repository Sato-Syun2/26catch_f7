/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    microros_app.c
  * @brief   catch26_interface を使用した micro-ROS アプリケーション
  ******************************************************************************
  */
/* USER CODE END Header */

#include "microros_app.h"
#include "ArmPositionMpc.h"
#include "command_auto_enable.h"

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
#include "C620Commission.h"
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
/* Reliable応答のACK待ちで100msのモーター指令監視を止めない。 */
#define MICROROS_SERVICE_ACK_TIMEOUT_MS 10
_Static_assert(MICROROS_SERVICE_ACK_TIMEOUT_MS < MICROROS_COMMAND_TIMEOUT_MS,
               "service ACK wait must be shorter than motor watchdog");

/* UrosF7Param.mode の共通値。速度系の拡張モードもサービスから選択できる。 */
#define MICROROS_MODE_POSITION        0U
#define MICROROS_MODE_VELOCITY        1U
#define MICROROS_MODE_CURRENT         2U
#define MICROROS_MODE_POSITION_AW     3U
#define MICROROS_MODE_VELOCITY_DOB    4U
#define MICROROS_MODE_POSITION_MPC    5U
#define MICROROS_MODE_MAX             MICROROS_MODE_POSITION_MPC

typedef enum {
  MICROROS_SERVICE_ACTION_MODE = 0,
  MICROROS_SERVICE_ACTION_ENABLE,
  MICROROS_SERVICE_ACTION_DISABLE
} microros_service_action_t;

typedef enum {
  MICROROS_SERVICE_COMMAND_OK = 0,
  MICROROS_SERVICE_COMMAND_INVALID,
  MICROROS_SERVICE_COMMAND_INVALID_MODE
} microros_service_command_result_t;

typedef struct {
  const char *name;
  microros_service_action_t action;
  uint8_t fixed_mode;
  bool mode_from_data;
} microros_service_command_definition_t;

/* modeを正規形式とし、既存の個別モード名は互換エイリアスとして残す。 */
static const microros_service_command_definition_t
    microros_service_command_definitions[] = {
      {"mode", MICROROS_SERVICE_ACTION_MODE, 0U, true},
      {"position_aw", MICROROS_SERVICE_ACTION_MODE,
       MICROROS_MODE_POSITION_AW, false},
      {"pos_aw", MICROROS_SERVICE_ACTION_MODE,
       MICROROS_MODE_POSITION_AW, false},
      {"posision_aw", MICROROS_SERVICE_ACTION_MODE,
       MICROROS_MODE_POSITION_AW, false},
      {"velocity_dob", MICROROS_SERVICE_ACTION_MODE,
       MICROROS_MODE_VELOCITY_DOB, false},
      {"vel_dob", MICROROS_SERVICE_ACTION_MODE,
       MICROROS_MODE_VELOCITY_DOB, false},
      {"position_mpc", MICROROS_SERVICE_ACTION_MODE,
       MICROROS_MODE_POSITION_MPC, false},
      {"enable", MICROROS_SERVICE_ACTION_ENABLE, 0U, false},
      {"disable", MICROROS_SERVICE_ACTION_DISABLE, 0U, false}
    };

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
/* 初期化後の最初のROS指令をEnableの解禁条件にする。 */
static volatile bool
    robstride_waiting_for_first_command[ROBSTRIDE_DEVICE_STORAGE_COUNT];
static volatile uint32_t
    robstride_command_last_tick[ROBSTRIDE_DEVICE_STORAGE_COUNT];
static volatile uint32_t
    robomas_command_last_tick[ROBOMAS_DEVICE_STORAGE_COUNT];
/* まだAgentへ接続していない場合も、ROS指令なしとして安全側へ倒す。 */
static volatile bool microros_command_watchdog_initialized = true;
static volatile bool microros_control_transaction_active = false;
static volatile uint32_t
    robstride_command_generation[ROBSTRIDE_DEVICE_STORAGE_COUNT];

/* 受信コールバックでは数えるだけにし、UART出力は低頻度のタスク側で行う。 */
static volatile uint32_t microros_command_received_count = 0U;
static volatile uint32_t microros_command_coalesced_count = 0U;
static uint32_t microros_diagnostics_last_tick = 0U;
static bool microros_diagnostics_started = false;
/* UARTを増やさず周期を測るRAMログ。停止後SWDから取り出す。 */
typedef struct {
  uint32_t tick, interval, received, coalesced, ring_overrun, priority_full;
  uint32_t tx_errors, can_errors, can_error_code, loops, loop_gap_ms, enabled_mask;
  Robstride_RateCounters can;
} ArmRateSample;
volatile struct { uint32_t count; ArmRateSample samples[120]; } arm_rate_log;
static uint32_t arm_rate_loops, arm_rate_loop_gap, arm_rate_last_loop;
static uint32_t microros_last_feedback_error_tick = 0U;

static void command_callback(const void *msgin);
static void parameter_service_callback(const void *request_msg,
                                       void *response_msg);
static void feedback_timer_callback(rcl_timer_t *timer, int64_t last_call_time);

static bool begin_control_transaction(void);
static void end_control_transaction(void);
static void reset_robstride_command_watchdog(uint32_t index);
static void reset_robomas_command_watchdog(uint32_t index);
static bool enable_robstride_for_command(uint32_t index, uint32_t generation);
static void microros_delay(uint32_t milliseconds);
static bool command_watchdog_expired(const volatile uint32_t *last_tick);
static bool robstride_timeout_mode(ROBSTRIDE_CTRL_TYPE ctrl_type);
static bool robomas_timeout_mode(ROBOMAS_CTRL_TYPE ctrl_type);

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

static bool begin_control_transaction(void)
{
  const uint32_t primask = __get_PRIMASK();
  bool acquired;

  __disable_irq();
  acquired = !microros_control_transaction_active;
  if (acquired) {
    microros_control_transaction_active = true;
  }
  __set_PRIMASK(primask);
  return acquired;
}

static void end_control_transaction(void)
{
  const uint32_t primask = __get_PRIMASK();

  __disable_irq();
  microros_control_transaction_active = false;
  __set_PRIMASK(primask);
}

static bool command_watchdog_expired(const volatile uint32_t *last_tick)
{
  const uint32_t primask = __get_PRIMASK();
  uint32_t previous_tick;
  bool initialized;

  __disable_irq();
  initialized = microros_command_watchdog_initialized;
  previous_tick = *last_tick;
  /* 受信時刻と現在時刻を同じ区間で取得する。呼出側の古いnowを使うと、
   * 新着指令の時刻との差がunsignedで桁あふれし、即時タイムアウトになる。 */
  const uint32_t now = HAL_GetTick();
  __set_PRIMASK(primask);

  return initialized &&
         ((uint32_t)(now - previous_tick) >= MICROROS_COMMAND_TIMEOUT_MS);
}

static void reset_robstride_command_watchdog(const uint32_t index)
{
  if (index >= ROBSTRIDE_DEVICE_COUNT) {
    return;
  }

  const uint32_t now = HAL_GetTick();
  const uint32_t primask = __get_PRIMASK();

  __disable_irq();
  robstride_command_last_tick[index] = now;
  __set_PRIMASK(primask);
}

static void reset_robomas_command_watchdog(const uint32_t index)
{
  if (index >= ROBOMAS_DEVICE_COUNT) {
    return;
  }

  const uint32_t now = HAL_GetTick();
  const uint32_t primask = __get_PRIMASK();

  __disable_irq();
  robomas_command_last_tick[index] = now;
  __set_PRIMASK(primask);
}

static bool enable_robstride_for_command(const uint32_t index, const uint32_t generation)
{
  Robstride_DeviceInfo *device;
  uint8_t enabled;

  if (index >= ROBSTRIDE_DEVICE_COUNT) {
    return false;
  }
  device = &robstride_dev_info_global[index];
  if (device->ctrl_param._enable_flag != 0U) {
    return true;
  }
  /* ログだけを間引く。正常状態へ戻った新着指令のEnableは遅延させない。 */
  static uint32_t last_report[ROBSTRIDE_DEVICE_STORAGE_COUNT];
  static bool reported[ROBSTRIDE_DEVICE_STORAGE_COUNT];
  const uint32_t now = HAL_GetTick();
  const char *blocked = Robstride_EnableBlockedReason(device);
  if (blocked) {
    if (!reported[index] || (uint32_t)(now-last_report[index]) >= 1000U) {
      printf("[micro-ROS] Robstride ID %u auto-enable blocked: %s\r\n",
             (unsigned)device->device_id, blocked);
      last_report[index] = now;
      reported[index] = true;
    }
    return false;
  }
  if (!begin_control_transaction()) {
    return false;
  }

  /* Disable/モード変更で破棄された指令ではトルクを入れない。 */
  if (!robstride_command_valid[index] ||
      Robstride_GetTargetGeneration(device) != generation) {
    end_control_transaction();
    return false;
  }

  printf("[micro-ROS] Robstride ID %u command; auto-enabling\r\n",
         (unsigned int)device->device_id);
  enabled = Robstride_ControlEnable(device, microros_delay);
  printf("[micro-ROS] Robstride ID %u command auto-enable result=%u\r\n",
         (unsigned int)device->device_id,
         (unsigned int)enabled);
  end_control_transaction();
  if (enabled) reported[index] = false;
  return enabled != 0U;
}

/* 位置制御は最後の目標を保持する。ROS指令欠落ではDisableしない。
 * 速度・電流制御のみ指令タイムアウトを監視する。 */
static bool robstride_timeout_mode(const ROBSTRIDE_CTRL_TYPE ctrl_type)
{
  return ctrl_type == ROBSTRIDE_CTRL_VEL ||
         ctrl_type == ROBSTRIDE_CTRL_VEL_DOB ||
         ctrl_type == ROBSTRIDE_CTRL_CURRENT;
}

static bool robomas_timeout_mode(const ROBOMAS_CTRL_TYPE ctrl_type)
{
  return ctrl_type == ROBOMAS_CTRL_VEL ||
         ctrl_type == ROBOMAS_CTRL_VEL_DOB ||
         ctrl_type == ROBOMAS_CTRL_CURRENT;
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
  if (!isfinite(data) || data < 0.0f || data > (float)MICROROS_MODE_MAX) {
    return false;
  }

  const uint8_t parsed_mode = (uint8_t)data;
  if ((float)parsed_mode != data) {
    return false;
  }
  *mode = parsed_mode;
  return true;
}

static microros_service_command_result_t parse_service_command(
    const rosidl_runtime_c__String *command,
    float data,
    microros_service_action_t *action,
    uint8_t *mode)
{
  if (command == NULL || action == NULL || mode == NULL) {
    return MICROROS_SERVICE_COMMAND_INVALID;
  }

  for (size_t i = 0U;
       i < (sizeof(microros_service_command_definitions) /
            sizeof(microros_service_command_definitions[0]));
       ++i) {
    const microros_service_command_definition_t *const definition =
        &microros_service_command_definitions[i];

    if (!ros_string_equals_literal(command, definition->name)) {
      continue;
    }

    *action = definition->action;
    if (!definition->mode_from_data) {
      *mode = definition->fixed_mode;
      return MICROROS_SERVICE_COMMAND_OK;
    }

    return parse_mode_data(data, mode)
               ? MICROROS_SERVICE_COMMAND_OK
               : MICROROS_SERVICE_COMMAND_INVALID_MODE;
  }

  return MICROROS_SERVICE_COMMAND_INVALID;
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
    case MICROROS_MODE_VELOCITY_DOB:
      *ctrl_type = ROBSTRIDE_CTRL_VEL_DOB;
      return true;
    case MICROROS_MODE_POSITION_MPC:
      *ctrl_type = ROBSTRIDE_CTRL_POS_MPC;
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
    case MICROROS_MODE_POSITION_AW:
      *ctrl_type = ROBOMAS_CTRL_POS_AW;
      return true;
    case MICROROS_MODE_POSITION_MPC:
      *ctrl_type = ROBOMAS_CTRL_POS_MPC;
      return true;
    case MICROROS_MODE_VELOCITY_DOB:
      *ctrl_type = ROBOMAS_CTRL_VEL_DOB;
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
  Robstride_InvalidateTargetGeneration(device);
  __set_PRIMASK(primask);
}

static bool set_robstride_mode(Robstride_DeviceInfo *device, uint8_t mode)
{
  if(mode == MICROROS_MODE_POSITION_MPC && device->device_id!=1U && device->device_id!=2U) return false;
  ROBSTRIDE_CTRL_TYPE ctrl_type;
  bool changed;

  if (!map_robstride_mode(mode, &ctrl_type)) {
    return false;
  }

  /* モード遷移中に旧ROS目標を低優先度CANへ再投入させない。 */
  invalidate_robstride_command(device);
  changed = Robstride_ServiceChangeControl(device, ctrl_type, microros_delay) != 0U;
  if (changed && robstride_timeout_mode(ctrl_type)) {
    reset_robstride_command_watchdog(
        (uint32_t)(device - robstride_dev_info_global));
  }
  return changed;
}

static bool set_robomas_mode(RoboMas_DeviceInfo *device, uint8_t mode)
{
  /* ID4と、校正・端点確認ゲートを持つC620のみ。 */
  if (mode == MICROROS_MODE_POSITION_MPC && device->device_id != 4U &&
      device->device_type != ROBOMASTER_C620) return false;
  if (device->device_type == ROBOMASTER_C620 &&
      (device->ctrl_param._enable_flag || device->ctrl_param._is_calibrating || C620_IsSurvey()))
    return false;
  ROBOMAS_CTRL_TYPE ctrl_type;
  if (!map_robomas_mode(mode, &ctrl_type)) {
    return false;
  }

  const bool was_enabled = device->ctrl_param._enable_flag;
  RoboMas_ChangeControl(device, ctrl_type);
  if (was_enabled) {
    RoboMas_ControlEnable(device);
  }
  if (robomas_timeout_mode(ctrl_type)) {
    reset_robomas_command_watchdog(
        (uint32_t)(device - robomas_dev_info_global));
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

  microros_service_action_t action;
  /* C620の校正・端探索を通常のenableと分離する。文字列は受信上限内。 */
  if (target_type == MICROROS_TARGET_ROBOMASTER &&
      request->command.size >= 5U && request->command.data != NULL &&
      strncmp(request->command.data, "c620_", 5U) == 0) {
    const int index = find_robomas_device_by_id(device_id);
    if (index < 0 || !begin_control_transaction()) {
      set_parameter_response(response, false, "target missing/control busy");
      return;
    }
    char message[MICROROS_RESPONSE_STORAGE_SIZE];
    const bool ok = C620_Service(&robomas_dev_info_global[index],
                                request->command.data, request->data,
                                message, sizeof(message));
    if (ok && (strcmp(request->command.data, "c620_survey") == 0 ||
               strcmp(request->command.data, "c620_calibrate") == 0 ||
               strcmp(request->command.data, "c620_pulse") == 0))
      reset_robomas_command_watchdog((uint32_t)index);
    end_control_transaction();
    set_parameter_response(response, ok, message);
    return;
  }
  uint8_t mode;
  const microros_service_command_result_t command_result =
      parse_service_command(&request->command,
                            request->data,
                            &action,
                            &mode);
  if (command_result == MICROROS_SERVICE_COMMAND_INVALID) {
    set_parameter_response(response, false, "invalid command");
    return;
  }
  if (command_result == MICROROS_SERVICE_COMMAND_INVALID_MODE) {
    set_parameter_response(response, false, "mode data must be integer 0..5");
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

  if (action == MICROROS_SERVICE_ACTION_MODE) {
    if (!begin_control_transaction()) {
      set_parameter_response(response, false, "control busy");
      return;
    }
    const bool changed = (target_type == MICROROS_TARGET_ROBSTRIDE)
                           ? set_robstride_mode(
                               &robstride_dev_info_global[device_index], mode)
                           : set_robomas_mode(
                               &robomas_dev_info_global[device_index], mode);
    end_control_transaction();
    if (changed) {
      set_parameter_response(response, true, "mode changed");
    } else {
      set_parameter_response(response, false, "mode change failed");
    }
    return;
  }

  /* Enable/Disable は data を参照しない。 */
  if (!begin_control_transaction()) {
    set_parameter_response(response, false, "control busy");
    return;
  }

  const bool enabling = action == MICROROS_SERVICE_ACTION_ENABLE;
  const char *operation_error = NULL;
  if (target_type == MICROROS_TARGET_ROBSTRIDE) {
    Robstride_DeviceInfo *device = &robstride_dev_info_global[device_index];
    /* Disableでは古い目標値を破棄する。Enableでは受信済みの最新値を
     * 保持し、topicを1回送った後の再Enableでも適用できるようにする。 */
    if (!enabling) {
      invalidate_robstride_command(device);
      robstride_waiting_for_first_command[device_index] = false;
    }
    /* topic受信直後は制御タスクがpendingを処理する前でもEnableを許可する。
     * waitingフラグだけを見ると、受信済みなのにserviceが競合して拒否される。 */
    if (enabling && robstride_waiting_for_first_command[device_index] &&
        !robstride_command_valid[device_index]) {
      operation_error = "waiting for first ROS command";
    } else if (Read_Robstride_FeedbackData(device).get_flag == 0U) {
      operation_error = "motor disconnected";
    } else {
      const uint8_t control_ok =
          enabling ? Robstride_ControlEnable(device, microros_delay)
                   : Robstride_ControlDisable(device, microros_delay);
      if (!control_ok) {
        operation_error = enabling ? Robstride_EnableBlockedReason(device) : NULL;
        if (!operation_error) operation_error = enabling ? "enable timeout" : "disable timeout";
      }
    }
  } else {
    RoboMas_DeviceInfo *device = &robomas_dev_info_global[device_index];
    if (enabling) {
      RoboMas_ControlEnable(device);
      if (!device->ctrl_param._enable_flag) {
        operation_error = "enable blocked by safety guard";
      }
    } else {
      if (device->device_type == ROBOMASTER_C620) C620_CancelRequest();
      RoboMas_ControlDisable(device);
    }
  }

  end_control_transaction();
  if (operation_error != NULL) {
    set_parameter_response(response, false, operation_error);
    return;
  }

  if (enabling) {
    if (target_type == MICROROS_TARGET_ROBSTRIDE) {
      reset_robstride_command_watchdog((uint32_t)device_index);
    } else {
      reset_robomas_command_watchdog((uint32_t)device_index);
    }
  }
  set_parameter_response(response, true, enabling ? "enabled" : "disabled");
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
    case ROBSTRIDE_CTRL_POS_MPC:
      return command->position;
    case ROBSTRIDE_CTRL_VEL:
    case ROBSTRIDE_CTRL_VEL_DOB:
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
    case ROBOMAS_CTRL_POS_AW:
    case ROBOMAS_CTRL_POS_MPC:
      return command->position;
    case ROBOMAS_CTRL_VEL:
    case ROBOMAS_CTRL_VEL_DOB:
      return command->velocity;
    case ROBOMAS_CTRL_CURRENT:
      return command->current;
    default:
      return 0.0f;
  }
}

static void apply_robstride_command(
    Robstride_DeviceInfo *device,
    const catch26_interface__msg__UrosF7MotorUnitCommand *command,
    const uint32_t expected_generation)
{
  /* Disable中のROS値は保存するが、CANへは絶対に送らない。 */
  if (device->ctrl_param._enable_flag == 0U ||
      Read_Robstride_FeedbackData(device).mode_status !=
          ROBSTRIDE_STATE_ENABLE) {
    return;
  }

  /* Robstride_SetTarget() 内で位置指令から offset_pos を減算する。 */
  (void)Robstride_SetTargetIfGeneration(
      device,
      robstride_command_target_value(device, command),
      expected_generation);
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

  if (command == NULL) {
    return;
  }

  const uint32_t received_tick = HAL_GetTick();
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
      robstride_command_generation[robstride_index] =
          Robstride_GetTargetGeneration(
              &robstride_dev_info_global[robstride_index]);
      robstride_command_valid[robstride_index] = true;
      robstride_command_pending[robstride_index] = true;
      robstride_command_last_tick[robstride_index] = received_tick;
      __set_PRIMASK(primask);
    } else if (robomas_index >= 0) {
      const uint32_t primask = __get_PRIMASK();

      __disable_irq();
      robomas_command_last_tick[robomas_index] = received_tick;
      robomas_dev_info_global[robomas_index].ctrl_param._startup_hold = false;
      __set_PRIMASK(primask);
      apply_robomas_command(&robomas_dev_info_global[robomas_index], unit);
    }
  }
}

void MicroRos_ApplyPendingRobstrideCommands(void)
{
  for (uint32_t i = 0U; i < ROBSTRIDE_DEVICE_COUNT; ++i) {
    catch26_interface__msg__UrosF7MotorUnitCommand command = {0};
    uint32_t generation = 0U;
    bool pending;
    bool waiting_for_first_command;
    const uint32_t primask = __get_PRIMASK();

    /* pendingを先に消費してからCAN処理を行う。処理中の新着値は次周回へ残る。 */
    __disable_irq();
    pending = robstride_command_pending[i];
    waiting_for_first_command = robstride_waiting_for_first_command[i];
    if (pending) {
      command = robstride_commands[i];
      generation = robstride_command_generation[i];
      robstride_command_pending[i] = false;
    }
    __set_PRIMASK(primask);

    if (pending) {
      Robstride_DeviceInfo *const device = &robstride_dev_info_global[i];
      bool command_still_valid;
      const bool position_mode = device->ctrl_param.ctrl_type == ROBSTRIDE_CTRL_POS ||
                                 device->ctrl_param.ctrl_type == ROBSTRIDE_CTRL_POS_MPC;
      const float first_target = robstride_command_target_value(device, &command);
      if (!isfinite(first_target) ||
          (position_mode &&
           !ArmPositionMpc_TargetAllowedForDevice(device->device_id, first_target))) {
        continue;
      }

      /* 再通電などで実機だけ停止した場合も、SWのEnable履歴を信じ続けない。 */
      const Robstride_FeedbackData actual = Read_Robstride_FeedbackData(device);
      if (position_mode && actual.get_flag && actual.mode_status == ROBSTRIDE_STATE_DISABLE)
        device->ctrl_param._enable_flag = 0U;

      /* 有効な指令の到着を記録。位置モードは初回以外も再Enableする。 */
      if (waiting_for_first_command) {
        /* 自動Enableの成否によらず、指令未受信状態を解除する。 */
        const uint32_t unlock_primask = __get_PRIMASK();
        __disable_irq();
        robstride_waiting_for_first_command[i] = false;
        __set_PRIMASK(unlock_primask);
      }
      /* Disable/タイムアウト後も新しい位置指令で復帰する。
       * 古い保持目標だけでは起動せず、Enable側の世代・FB・保護確認は維持。 */
      if (CommandAutoEnable_Required(waiting_for_first_command, position_mode,
                                    device->ctrl_param._enable_flag != 0U) &&
          !enable_robstride_for_command(i, generation)) {
        continue;
      }

      const uint32_t valid_primask = __get_PRIMASK();
      __disable_irq();
      command_still_valid = robstride_command_valid[i];
      __set_PRIMASK(valid_primask);
      if (!command_still_valid) {
        continue;
      }

      if (waiting_for_first_command) {
        const uint32_t unlock_primask = __get_PRIMASK();
        __disable_irq();
        robstride_waiting_for_first_command[i] = false;
        __set_PRIMASK(unlock_primask);
      }

      /* Enable成功後だけ目標値を適用する。 */
      /* DOBの状態更新はRefreshだけで行う。受信頻度依存の二重積分を避ける。 */
      if (device->ctrl_param._enable_flag != 0U &&
          device->ctrl_param.ctrl_type != ROBSTRIDE_CTRL_VEL_DOB &&
          device->ctrl_param.ctrl_type != ROBSTRIDE_CTRL_POS_MPC) {
        apply_robstride_command(device, &command, generation);
      }
    }
  }
}

void MicroRos_RefreshRobstrideTargets(void)
{
  const uint32_t rate_now=HAL_GetTick();
  const uint32_t rate_gap=rate_now-arm_rate_last_loop;
  if (arm_rate_last_loop && rate_gap>arm_rate_loop_gap) arm_rate_loop_gap=rate_gap;
  arm_rate_last_loop=rate_now;
  ++arm_rate_loops;
  /* 目標の有無に関係なく監視。通信失敗時はDisableを再試行する。 */
  static bool stop_pending[ROBSTRIDE_DEVICE_STORAGE_COUNT];
  for (uint32_t i = 0U; i < ROBSTRIDE_DEVICE_COUNT; ++i) {
    Robstride_DeviceInfo *device = &robstride_dev_info_global[i];
    if (device->ctrl_param._enable_flag && !Robstride_ArmGuardCheck(device)) stop_pending[i] = true;
    if (stop_pending[i]) {
      if (!begin_control_transaction()) continue;
      stop_pending[i] = !Robstride_ControlDisable(device, microros_delay);
      invalidate_robstride_command(device);
      end_control_transaction();
    }
  }
  static uint8_t normal_target_refresh_divider = 0U;
  bool refresh_normal_targets;

  ++normal_target_refresh_divider;
  refresh_normal_targets = normal_target_refresh_divider >= 5U;
  if (refresh_normal_targets) {
    normal_target_refresh_divider = 0U;
  }

  for (uint32_t i = 0U; i < ROBSTRIDE_DEVICE_COUNT; ++i) {
    catch26_interface__msg__UrosF7MotorUnitCommand command = {0};
    uint32_t generation = 0U;
    bool valid;
    const bool velocity_dob =
        robstride_dev_info_global[i].ctrl_param.ctrl_type ==
        ROBSTRIDE_CTRL_VEL_DOB ||
        robstride_dev_info_global[i].ctrl_param.ctrl_type == ROBSTRIDE_CTRL_POS_MPC;
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    valid = robstride_command_valid[i];
    if (valid) {
      command = robstride_commands[i];
      generation = robstride_command_generation[i];
    }
    __set_PRIMASK(primask);

    /* ROS指令を一度も受信していない間は目標値を適用しない。 */
    if (velocity_dob && valid &&
        robstride_dev_info_global[i].ctrl_param._enable_flag != 0U) {
      const float target = robstride_command_target_value(
          &robstride_dev_info_global[i], &command);
      (void)Robstride_SetTargetIfGeneration(&robstride_dev_info_global[i], target, generation);
    } else if (valid && refresh_normal_targets &&
               robstride_dev_info_global[i].ctrl_param._enable_flag != 0U) {
      apply_robstride_command(&robstride_dev_info_global[i], &command, generation);
    }
  }
}

void MicroRos_CheckRobstrideCommandTimeout(void)
{

  for (uint32_t i = 0U; i < ROBSTRIDE_DEVICE_COUNT; ++i) {
    Robstride_DeviceInfo *const device = &robstride_dev_info_global[i];

    if (!device->ctrl_param.ros_topic_timeout_enable ||
        device->ctrl_param._enable_flag == 0U ||
        !robstride_timeout_mode(device->ctrl_param.ctrl_type) ||
        !command_watchdog_expired(&robstride_command_last_tick[i])) {
      continue;
    }

    /* Disable／mode変更／EnableのCANトランザクションとは同時に実行しない。 */
    if (!begin_control_transaction()) {
      return;
    }

    if (device->ctrl_param.ros_topic_timeout_enable &&
        device->ctrl_param._enable_flag != 0U &&
        robstride_timeout_mode(device->ctrl_param.ctrl_type) &&
        command_watchdog_expired(&robstride_command_last_tick[i])) {
      invalidate_robstride_command(device);
      printf("[micro-ROS] Robstride ID %u command timeout; disabled\r\n",
             (unsigned int)device->device_id);
      (void)Robstride_ControlDisable(device, microros_delay);
    }

    end_control_transaction();
  }
}

void MicroRos_CheckRobomasCommandTimeout(void)
{

  for (uint32_t i = 0U; i < ROBOMAS_DEVICE_COUNT; ++i) {
    RoboMas_DeviceInfo *const device = &robomas_dev_info_global[i];

    if (device->ctrl_param._startup_hold || !device->ctrl_param.ros_topic_timeout_enable ||
        device->ctrl_param._enable_flag == 0U ||
        device->ctrl_param._is_calibrating ||
        !robomas_timeout_mode(device->ctrl_param.ctrl_type) ||
        !command_watchdog_expired(&robomas_command_last_tick[i])) {
      continue;
    }

    /* Disable／mode変更／EnableのCANトランザクションとは同時に実行しない。 */
    if (!begin_control_transaction()) {
      return;
    }

    if (device->ctrl_param.ros_topic_timeout_enable &&
        device->ctrl_param._enable_flag != 0U &&
        !device->ctrl_param._is_calibrating &&
        robomas_timeout_mode(device->ctrl_param.ctrl_type) &&
        command_watchdog_expired(&robomas_command_last_tick[i])) {
      if (device->device_id == 4U &&
          device->ctrl_param.ctrl_type == ROBOMAS_CTRL_VEL_DOB) {
        /* 新しい指令が来ても減速ラッチは停止完了まで解除しない。 */
        RoboMas_RequestId4VelocityBrake();
      } else {
        printf("[micro-ROS] RoboMaster ID %u command timeout; disabled\r\n",
               (unsigned int)device->device_id);
        RoboMas_ControlDisable(device);
      }
    }

    end_control_transaction();
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
  const uint32_t interval=now-microros_diagnostics_last_tick;
  microros_diagnostics_last_tick = now;

  const uint32_t received = take_counter(&microros_command_received_count);
  const uint32_t coalesced = take_counter(&microros_command_coalesced_count);
  const uint32_t ring_overrun = Robstride_TakeTxRingOverrunCount();
  const uint32_t priority_queue_full = Robstride_TakePriorityQueueFullCount();
  const uint32_t tx_errors = Robstride_TakeTxErrorCount();
  const uint32_t can_errors = Robstride_TakeCanErrorCount();
  const uint32_t can_error_code = Robstride_TakeCanErrorCode();
  uint32_t rate_enabled=0U;
  for (uint32_t i=0U;i<ROBSTRIDE_DEVICE_COUNT;++i) {
    const Robstride_DeviceInfo *dev=&robstride_dev_info_global[i];
    if (dev->device_id>=1U && dev->device_id<=2U && dev->ctrl_param._enable_flag)
      rate_enabled |= 1U<<(dev->device_id-1U);
  }
  const ArmRateSample rate_sample={now,interval,received,coalesced,ring_overrun,
    priority_queue_full,tx_errors,can_errors,can_error_code,arm_rate_loops,
    arm_rate_loop_gap,rate_enabled,Robstride_TakeRateCounters()};
  arm_rate_log.samples[arm_rate_log.count%120U]=rate_sample;
  ++arm_rate_log.count;
  arm_rate_loops=0U; arm_rate_loop_gap=0U;

  /* UART送信は1文字ごとに待つため、500Hz制御タスクでは実行しない。 */
}

static void print_control_diagnostics(void)
{
  static uint32_t printed_count;
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  const uint32_t count = arm_rate_log.count;
  if (count == printed_count) {__set_PRIMASK(mask); return;}
  const ArmRateSample sample = arm_rate_log.samples[(count - 1U) % 120U];
  printed_count = count;
  __set_PRIMASK(mask);
  const uint32_t received = sample.received, coalesced = sample.coalesced;
  const uint32_t ring_overrun = sample.ring_overrun, priority_queue_full = sample.priority_full;
  const uint32_t tx_errors = sample.tx_errors, can_errors = sample.can_errors;
  const uint32_t can_error_code = sample.can_error_code;
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

  switch (device->ctrl_param.ctrl_type) {
    case ROBSTRIDE_CTRL_POS:
    case ROBSTRIDE_CTRL_POS_MPC:
      return catch26_interface__msg__UrosF7MotorUnitFeedback__STATE_POSITION;
    case ROBSTRIDE_CTRL_VEL:
    case ROBSTRIDE_CTRL_VEL_DOB:
      return catch26_interface__msg__UrosF7MotorUnitFeedback__STATE_VELOCITY;
    case ROBSTRIDE_CTRL_CURRENT:
      return catch26_interface__msg__UrosF7MotorUnitFeedback__STATE_CURRENT;
    default:
      break;
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
  /* 通常はmechPos、DOB運転中はType2専用キャッシュ。非同期混入を防ぐ。 */
  bool position_fresh = true;
  if (CanDevices_IsInitialized()) {
    float position;
    uint32_t tick;
    const bool valid=Robstride_ReadMeasuredPosition(device,&position,&tick);
    if (valid) output->position=position;
    position_fresh=valid && (uint32_t)(HAL_GetTick()-tick)<=50U;
  }
  output->velocity = feedback.velocity;
  output->current = feedback.current;
  if (Robstride_UsesStandardFeedback(device)) {
    Robstride_StandardFeedback standard;
    if (Robstride_ReadStandardFeedback(device,&standard)) {
      output->position=standard.position;
      output->velocity=standard.velocity;
      /* DOB運転時のみ参考電流。正確なiqfは明示Type17読み取りで確認する。 */
      output->current=Robstride_StandardReferenceCurrent(device,&standard);
      position_fresh=(uint32_t)(HAL_GetTick()-standard.tick)<=50U;
    } else position_fresh=false;
  }
  output->state = robstride_state(device, &feedback);
  output->unit_message_code = feedback.get_flag && position_fresh
                                  ? catch26_interface__msg__UrosF7MotorUnitFeedback__CODE_NORMAL
                                  : catch26_interface__msg__UrosF7MotorUnitFeedback__CODE_DISCONNECTION;
}

static void set_robomas_feedback(
    catch26_interface__msg__UrosF7MotorUnitFeedback *output,
    uint32_t index)
{
  const RoboMas_DeviceInfo *device = &robomas_dev_info_global[index];
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  const RoboMas_FeedbackData snapshot = robomas_fb[index];
  __set_PRIMASK(mask);
  const RoboMas_FeedbackData *feedback = &snapshot;

  output->info.type = catch26_interface__msg__DeviceInfo__TYPE_ROBOMASTER;
  output->info.id = device->device_id;
  output->position = feedback->position;
  output->velocity = feedback->velocity;
  output->current = feedback->current;
  if (device->ctrl_param._enable_flag == 0U) {
    output->state =
        catch26_interface__msg__UrosF7MotorUnitFeedback__STATE_DISABLE;
  } else {
    switch (device->ctrl_param.ctrl_type) {
      case ROBOMAS_CTRL_POS:
      case ROBOMAS_CTRL_POS_AW:
      case ROBOMAS_CTRL_POS_MPC:
        output->state =
            catch26_interface__msg__UrosF7MotorUnitFeedback__STATE_POSITION;
        break;
      case ROBOMAS_CTRL_VEL:
      case ROBOMAS_CTRL_VEL_DOB:
        output->state =
            catch26_interface__msg__UrosF7MotorUnitFeedback__STATE_VELOCITY;
        break;
      case ROBOMAS_CTRL_CURRENT:
        output->state =
            catch26_interface__msg__UrosF7MotorUnitFeedback__STATE_CURRENT;
        break;
      default:
        output->state =
            catch26_interface__msg__UrosF7MotorUnitFeedback__STATE_DISABLE;
        break;
    }
  }
  output->unit_message_code = feedback->get_flag
                                ? catch26_interface__msg__UrosF7MotorUnitFeedback__CODE_NORMAL
                                : catch26_interface__msg__UrosF7MotorUnitFeedback__CODE_DISCONNECTION;
}

static void feedback_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  (void)timer;
  (void)last_call_time;

  RoboMas_PrintId4Diagnostic();

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
  const uint32_t now = HAL_GetTick();
  const uint32_t primask = __get_PRIMASK();

  __disable_irq();
  for (uint32_t i = 0U; i < ROBSTRIDE_DEVICE_STORAGE_COUNT; ++i) {
    robstride_command_valid[i] = false;
    robstride_command_pending[i] = false;
    robstride_waiting_for_first_command[i] = true;
    robstride_command_generation[i] = 0U;
    robstride_commands[i] = (catch26_interface__msg__UrosF7MotorUnitCommand){0};
    robstride_command_last_tick[i] = now;
  }
  for (uint32_t i = 0U; i < ROBOMAS_DEVICE_STORAGE_COUNT; ++i) {
    robomas_command_last_tick[i] = now;
  }
  microros_command_received_count = 0U;
  microros_command_coalesced_count = 0U;
  microros_command_watchdog_initialized = true;
  microros_control_transaction_active = false;
  __set_PRIMASK(primask);
}

/* 通信の一時的な欠落は許容し、Agent再起動で失われたセッションは作り直す。
 * pingもexecutorと同じタスクで実行し、transportへの同時アクセスを避ける。 */
static bool agent_session_lost(uint32_t *last_check, uint8_t *failures)
{
  const uint32_t now = HAL_GetTick();
  if ((uint32_t)(now - *last_check) < 1000U) {
    return false;
  }
  *last_check = now;
  if (rmw_uros_ping_agent(10, 1) == RMW_RET_OK) {
    *failures = 0U;
    return false;
  }
  return ++(*failures) >= 3U;
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
    /* 各送信元は別モーターの部分指令を送る。depth=1ではShoot/Transfer/XYが
     * 同じ受信周期に届いた際、最後の一件以外が消えwatchdogが誤作動する。
     * 静的ライブラリが確保済みの全履歴を使い、軸別の最新値化はcallbackで行う。 */
    command_qos.depth = RMW_UXRCE_MAX_HISTORY;
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
    /* 既定の1000ms待ちは同じexecutor上の指令受信/feedbackを停止する。
     * Reliable QoSは維持し、同期ACK待ちだけを制御周期内に制限する。
     * 未確認の応答はXRCEのReliableストリームに残り、後続spinで処理する。 */
    RCCHECK(rmw_uros_set_service_session_timeout(
        rcl_service_get_rmw_handle(&parameter_service),
        MICROROS_SERVICE_ACK_TIMEOUT_MS));
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
    uint32_t last_agent_check = HAL_GetTick();
    uint8_t agent_ping_failures = 0U;

    for (;;) {
      const rcl_ret_t spin_result =
          rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));
      if (spin_result != RCL_RET_OK && spin_result != RCL_RET_TIMEOUT) {
        printf("micro-ROS executor stopped: %d\r\n", (int)spin_result);
        rcl_reset_error();
        break;
      }
      if (agent_session_lost(&last_agent_check, &agent_ping_failures)) {
        printf("micro-ROS agent session lost; reconnecting\r\n");
        break;
      }
      print_control_diagnostics();
      osDelay(1U);
    }

    /* 消失したAgentからの破棄ACKを待たず、既存リソースを解放する。 */
    (void)rmw_uros_set_context_entity_destroy_session_timeout(
        rcl_context_get_rmw_context(&support.context), 0);
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

// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from catch26_interface:msg/UrosF7MotorUnitFeedback.idl
// generated code does not contain a copyright notice

#ifndef CATCH26_INTERFACE__MSG__DETAIL__UROS_F7_MOTOR_UNIT_FEEDBACK__STRUCT_H_
#define CATCH26_INTERFACE__MSG__DETAIL__UROS_F7_MOTOR_UNIT_FEEDBACK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'STATE_DISABLE'.
enum
{
  catch26_interface__msg__UrosF7MotorUnitFeedback__STATE_DISABLE = 0
};

/// Constant 'STATE_POSITION'.
enum
{
  catch26_interface__msg__UrosF7MotorUnitFeedback__STATE_POSITION = 1
};

/// Constant 'STATE_VELOCITY'.
enum
{
  catch26_interface__msg__UrosF7MotorUnitFeedback__STATE_VELOCITY = 2
};

/// Constant 'STATE_CURRENT'.
enum
{
  catch26_interface__msg__UrosF7MotorUnitFeedback__STATE_CURRENT = 3
};

/// Constant 'CODE_NONE'.
enum
{
  catch26_interface__msg__UrosF7MotorUnitFeedback__CODE_NONE = 0
};

/// Constant 'CODE_NORMAL'.
enum
{
  catch26_interface__msg__UrosF7MotorUnitFeedback__CODE_NORMAL = 1
};

/// Constant 'CODE_DISCONNECTION'.
enum
{
  catch26_interface__msg__UrosF7MotorUnitFeedback__CODE_DISCONNECTION = 2
};

/// Constant 'CODE_ERROR'.
enum
{
  catch26_interface__msg__UrosF7MotorUnitFeedback__CODE_ERROR = 3
};

// Include directives for member types
// Member 'info'
#include "catch26_interface/msg/detail/device_info__struct.h"

/// Struct defined in msg/UrosF7MotorUnitFeedback in the package catch26_interface.
typedef struct catch26_interface__msg__UrosF7MotorUnitFeedback
{
  catch26_interface__msg__DeviceInfo info;
  float position;
  float velocity;
  float current;
  uint8_t state;
  uint8_t unit_message_code;
} catch26_interface__msg__UrosF7MotorUnitFeedback;

// Struct for a sequence of catch26_interface__msg__UrosF7MotorUnitFeedback.
typedef struct catch26_interface__msg__UrosF7MotorUnitFeedback__Sequence
{
  catch26_interface__msg__UrosF7MotorUnitFeedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} catch26_interface__msg__UrosF7MotorUnitFeedback__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CATCH26_INTERFACE__MSG__DETAIL__UROS_F7_MOTOR_UNIT_FEEDBACK__STRUCT_H_

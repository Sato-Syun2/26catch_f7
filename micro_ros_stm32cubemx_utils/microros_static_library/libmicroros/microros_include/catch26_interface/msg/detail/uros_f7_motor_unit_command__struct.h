// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from catch26_interface:msg/UrosF7MotorUnitCommand.idl
// generated code does not contain a copyright notice

#ifndef CATCH26_INTERFACE__MSG__DETAIL__UROS_F7_MOTOR_UNIT_COMMAND__STRUCT_H_
#define CATCH26_INTERFACE__MSG__DETAIL__UROS_F7_MOTOR_UNIT_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "catch26_interface/msg/detail/device_info__struct.h"

/// Struct defined in msg/UrosF7MotorUnitCommand in the package catch26_interface.
typedef struct catch26_interface__msg__UrosF7MotorUnitCommand
{
  catch26_interface__msg__DeviceInfo info;
  float position;
  float velocity;
  float current;
  uint8_t unit_option;
} catch26_interface__msg__UrosF7MotorUnitCommand;

// Struct for a sequence of catch26_interface__msg__UrosF7MotorUnitCommand.
typedef struct catch26_interface__msg__UrosF7MotorUnitCommand__Sequence
{
  catch26_interface__msg__UrosF7MotorUnitCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} catch26_interface__msg__UrosF7MotorUnitCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CATCH26_INTERFACE__MSG__DETAIL__UROS_F7_MOTOR_UNIT_COMMAND__STRUCT_H_

// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from catch26_interface:msg/UrosF7Command.idl
// generated code does not contain a copyright notice

#ifndef CATCH26_INTERFACE__MSG__DETAIL__UROS_F7_COMMAND__STRUCT_H_
#define CATCH26_INTERFACE__MSG__DETAIL__UROS_F7_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'command'
#include "catch26_interface/msg/detail/uros_f7_motor_unit_command__struct.h"

// constants for array fields with an upper bound
// command
enum
{
  catch26_interface__msg__UrosF7Command__command__MAX_SIZE = 8
};

/// Struct defined in msg/UrosF7Command in the package catch26_interface.
typedef struct catch26_interface__msg__UrosF7Command
{
  catch26_interface__msg__UrosF7MotorUnitCommand__Sequence command;
  uint8_t option;
} catch26_interface__msg__UrosF7Command;

// Struct for a sequence of catch26_interface__msg__UrosF7Command.
typedef struct catch26_interface__msg__UrosF7Command__Sequence
{
  catch26_interface__msg__UrosF7Command * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} catch26_interface__msg__UrosF7Command__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CATCH26_INTERFACE__MSG__DETAIL__UROS_F7_COMMAND__STRUCT_H_

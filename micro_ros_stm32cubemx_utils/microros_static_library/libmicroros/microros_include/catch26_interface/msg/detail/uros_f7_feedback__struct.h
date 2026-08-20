// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from catch26_interface:msg/UrosF7Feedback.idl
// generated code does not contain a copyright notice

#ifndef CATCH26_INTERFACE__MSG__DETAIL__UROS_F7_FEEDBACK__STRUCT_H_
#define CATCH26_INTERFACE__MSG__DETAIL__UROS_F7_FEEDBACK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'local_time'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'feedback'
#include "catch26_interface/msg/detail/uros_f7_motor_unit_feedback__struct.h"
// Member 'last_command'
#include "catch26_interface/msg/detail/uros_f7_command__struct.h"
// Member 'optional_datas'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'message'
#include "rosidl_runtime_c/string.h"

// constants for array fields with an upper bound
// feedback
enum
{
  catch26_interface__msg__UrosF7Feedback__feedback__MAX_SIZE = 8
};
// optional_datas
enum
{
  catch26_interface__msg__UrosF7Feedback__optional_datas__MAX_SIZE = 8
};
// message
enum
{
  catch26_interface__msg__UrosF7Feedback__message__MAX_STRING_SIZE = 32
};

/// Struct defined in msg/UrosF7Feedback in the package catch26_interface.
typedef struct catch26_interface__msg__UrosF7Feedback
{
  builtin_interfaces__msg__Time local_time;
  catch26_interface__msg__UrosF7MotorUnitFeedback__Sequence feedback;
  catch26_interface__msg__UrosF7Command last_command;
  rosidl_runtime_c__float__Sequence optional_datas;
  rosidl_runtime_c__String message;
} catch26_interface__msg__UrosF7Feedback;

// Struct for a sequence of catch26_interface__msg__UrosF7Feedback.
typedef struct catch26_interface__msg__UrosF7Feedback__Sequence
{
  catch26_interface__msg__UrosF7Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} catch26_interface__msg__UrosF7Feedback__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CATCH26_INTERFACE__MSG__DETAIL__UROS_F7_FEEDBACK__STRUCT_H_

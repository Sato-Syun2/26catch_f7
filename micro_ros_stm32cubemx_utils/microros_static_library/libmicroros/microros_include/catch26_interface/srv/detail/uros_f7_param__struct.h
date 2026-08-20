// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from catch26_interface:srv/UrosF7Param.idl
// generated code does not contain a copyright notice

#ifndef CATCH26_INTERFACE__SRV__DETAIL__UROS_F7_PARAM__STRUCT_H_
#define CATCH26_INTERFACE__SRV__DETAIL__UROS_F7_PARAM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'target'
// Member 'command'
#include "rosidl_runtime_c/string.h"

// constants for array fields with an upper bound
// target
enum
{
  catch26_interface__srv__UrosF7Param_Request__target__MAX_STRING_SIZE = 16
};
// command
enum
{
  catch26_interface__srv__UrosF7Param_Request__command__MAX_STRING_SIZE = 16
};

/// Struct defined in srv/UrosF7Param in the package catch26_interface.
typedef struct catch26_interface__srv__UrosF7Param_Request
{
  rosidl_runtime_c__String target;
  rosidl_runtime_c__String command;
  float data;
} catch26_interface__srv__UrosF7Param_Request;

// Struct for a sequence of catch26_interface__srv__UrosF7Param_Request.
typedef struct catch26_interface__srv__UrosF7Param_Request__Sequence
{
  catch26_interface__srv__UrosF7Param_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} catch26_interface__srv__UrosF7Param_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

// constants for array fields with an upper bound
// message
enum
{
  catch26_interface__srv__UrosF7Param_Response__message__MAX_STRING_SIZE = 32
};

/// Struct defined in srv/UrosF7Param in the package catch26_interface.
typedef struct catch26_interface__srv__UrosF7Param_Response
{
  bool success;
  rosidl_runtime_c__String message;
} catch26_interface__srv__UrosF7Param_Response;

// Struct for a sequence of catch26_interface__srv__UrosF7Param_Response.
typedef struct catch26_interface__srv__UrosF7Param_Response__Sequence
{
  catch26_interface__srv__UrosF7Param_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} catch26_interface__srv__UrosF7Param_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CATCH26_INTERFACE__SRV__DETAIL__UROS_F7_PARAM__STRUCT_H_

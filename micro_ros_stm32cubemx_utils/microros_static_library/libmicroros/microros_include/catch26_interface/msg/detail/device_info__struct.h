// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from catch26_interface:msg/DeviceInfo.idl
// generated code does not contain a copyright notice

#ifndef CATCH26_INTERFACE__MSG__DETAIL__DEVICE_INFO__STRUCT_H_
#define CATCH26_INTERFACE__MSG__DETAIL__DEVICE_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'TYPE_ROBOMASTER'.
enum
{
  catch26_interface__msg__DeviceInfo__TYPE_ROBOMASTER = 0
};

/// Constant 'TYPE_ROBSTRIDE'.
enum
{
  catch26_interface__msg__DeviceInfo__TYPE_ROBSTRIDE = 1
};

/// Constant 'TYPE_KONDO'.
enum
{
  catch26_interface__msg__DeviceInfo__TYPE_KONDO = 2
};

/// Struct defined in msg/DeviceInfo in the package catch26_interface.
typedef struct catch26_interface__msg__DeviceInfo
{
  uint8_t type;
  uint8_t id;
} catch26_interface__msg__DeviceInfo;

// Struct for a sequence of catch26_interface__msg__DeviceInfo.
typedef struct catch26_interface__msg__DeviceInfo__Sequence
{
  catch26_interface__msg__DeviceInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} catch26_interface__msg__DeviceInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CATCH26_INTERFACE__MSG__DETAIL__DEVICE_INFO__STRUCT_H_

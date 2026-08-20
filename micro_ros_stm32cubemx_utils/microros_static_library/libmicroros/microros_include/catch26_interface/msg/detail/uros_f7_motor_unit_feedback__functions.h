// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from catch26_interface:msg/UrosF7MotorUnitFeedback.idl
// generated code does not contain a copyright notice

#ifndef CATCH26_INTERFACE__MSG__DETAIL__UROS_F7_MOTOR_UNIT_FEEDBACK__FUNCTIONS_H_
#define CATCH26_INTERFACE__MSG__DETAIL__UROS_F7_MOTOR_UNIT_FEEDBACK__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "catch26_interface/msg/rosidl_generator_c__visibility_control.h"

#include "catch26_interface/msg/detail/uros_f7_motor_unit_feedback__struct.h"

/// Initialize msg/UrosF7MotorUnitFeedback message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * catch26_interface__msg__UrosF7MotorUnitFeedback
 * )) before or use
 * catch26_interface__msg__UrosF7MotorUnitFeedback__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_catch26_interface
bool
catch26_interface__msg__UrosF7MotorUnitFeedback__init(catch26_interface__msg__UrosF7MotorUnitFeedback * msg);

/// Finalize msg/UrosF7MotorUnitFeedback message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_catch26_interface
void
catch26_interface__msg__UrosF7MotorUnitFeedback__fini(catch26_interface__msg__UrosF7MotorUnitFeedback * msg);

/// Create msg/UrosF7MotorUnitFeedback message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * catch26_interface__msg__UrosF7MotorUnitFeedback__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_catch26_interface
catch26_interface__msg__UrosF7MotorUnitFeedback *
catch26_interface__msg__UrosF7MotorUnitFeedback__create();

/// Destroy msg/UrosF7MotorUnitFeedback message.
/**
 * It calls
 * catch26_interface__msg__UrosF7MotorUnitFeedback__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_catch26_interface
void
catch26_interface__msg__UrosF7MotorUnitFeedback__destroy(catch26_interface__msg__UrosF7MotorUnitFeedback * msg);

/// Check for msg/UrosF7MotorUnitFeedback message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_catch26_interface
bool
catch26_interface__msg__UrosF7MotorUnitFeedback__are_equal(const catch26_interface__msg__UrosF7MotorUnitFeedback * lhs, const catch26_interface__msg__UrosF7MotorUnitFeedback * rhs);

/// Copy a msg/UrosF7MotorUnitFeedback message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_catch26_interface
bool
catch26_interface__msg__UrosF7MotorUnitFeedback__copy(
  const catch26_interface__msg__UrosF7MotorUnitFeedback * input,
  catch26_interface__msg__UrosF7MotorUnitFeedback * output);

/// Initialize array of msg/UrosF7MotorUnitFeedback messages.
/**
 * It allocates the memory for the number of elements and calls
 * catch26_interface__msg__UrosF7MotorUnitFeedback__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_catch26_interface
bool
catch26_interface__msg__UrosF7MotorUnitFeedback__Sequence__init(catch26_interface__msg__UrosF7MotorUnitFeedback__Sequence * array, size_t size);

/// Finalize array of msg/UrosF7MotorUnitFeedback messages.
/**
 * It calls
 * catch26_interface__msg__UrosF7MotorUnitFeedback__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_catch26_interface
void
catch26_interface__msg__UrosF7MotorUnitFeedback__Sequence__fini(catch26_interface__msg__UrosF7MotorUnitFeedback__Sequence * array);

/// Create array of msg/UrosF7MotorUnitFeedback messages.
/**
 * It allocates the memory for the array and calls
 * catch26_interface__msg__UrosF7MotorUnitFeedback__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_catch26_interface
catch26_interface__msg__UrosF7MotorUnitFeedback__Sequence *
catch26_interface__msg__UrosF7MotorUnitFeedback__Sequence__create(size_t size);

/// Destroy array of msg/UrosF7MotorUnitFeedback messages.
/**
 * It calls
 * catch26_interface__msg__UrosF7MotorUnitFeedback__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_catch26_interface
void
catch26_interface__msg__UrosF7MotorUnitFeedback__Sequence__destroy(catch26_interface__msg__UrosF7MotorUnitFeedback__Sequence * array);

/// Check for msg/UrosF7MotorUnitFeedback message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_catch26_interface
bool
catch26_interface__msg__UrosF7MotorUnitFeedback__Sequence__are_equal(const catch26_interface__msg__UrosF7MotorUnitFeedback__Sequence * lhs, const catch26_interface__msg__UrosF7MotorUnitFeedback__Sequence * rhs);

/// Copy an array of msg/UrosF7MotorUnitFeedback messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_catch26_interface
bool
catch26_interface__msg__UrosF7MotorUnitFeedback__Sequence__copy(
  const catch26_interface__msg__UrosF7MotorUnitFeedback__Sequence * input,
  catch26_interface__msg__UrosF7MotorUnitFeedback__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // CATCH26_INTERFACE__MSG__DETAIL__UROS_F7_MOTOR_UNIT_FEEDBACK__FUNCTIONS_H_

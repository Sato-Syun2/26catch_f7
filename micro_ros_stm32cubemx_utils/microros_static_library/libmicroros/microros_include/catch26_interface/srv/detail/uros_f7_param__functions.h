// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from catch26_interface:srv/UrosF7Param.idl
// generated code does not contain a copyright notice

#ifndef CATCH26_INTERFACE__SRV__DETAIL__UROS_F7_PARAM__FUNCTIONS_H_
#define CATCH26_INTERFACE__SRV__DETAIL__UROS_F7_PARAM__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "catch26_interface/msg/rosidl_generator_c__visibility_control.h"

#include "catch26_interface/srv/detail/uros_f7_param__struct.h"

/// Initialize srv/UrosF7Param message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * catch26_interface__srv__UrosF7Param_Request
 * )) before or use
 * catch26_interface__srv__UrosF7Param_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_catch26_interface
bool
catch26_interface__srv__UrosF7Param_Request__init(catch26_interface__srv__UrosF7Param_Request * msg);

/// Finalize srv/UrosF7Param message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_catch26_interface
void
catch26_interface__srv__UrosF7Param_Request__fini(catch26_interface__srv__UrosF7Param_Request * msg);

/// Create srv/UrosF7Param message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * catch26_interface__srv__UrosF7Param_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_catch26_interface
catch26_interface__srv__UrosF7Param_Request *
catch26_interface__srv__UrosF7Param_Request__create();

/// Destroy srv/UrosF7Param message.
/**
 * It calls
 * catch26_interface__srv__UrosF7Param_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_catch26_interface
void
catch26_interface__srv__UrosF7Param_Request__destroy(catch26_interface__srv__UrosF7Param_Request * msg);

/// Check for srv/UrosF7Param message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_catch26_interface
bool
catch26_interface__srv__UrosF7Param_Request__are_equal(const catch26_interface__srv__UrosF7Param_Request * lhs, const catch26_interface__srv__UrosF7Param_Request * rhs);

/// Copy a srv/UrosF7Param message.
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
catch26_interface__srv__UrosF7Param_Request__copy(
  const catch26_interface__srv__UrosF7Param_Request * input,
  catch26_interface__srv__UrosF7Param_Request * output);

/// Initialize array of srv/UrosF7Param messages.
/**
 * It allocates the memory for the number of elements and calls
 * catch26_interface__srv__UrosF7Param_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_catch26_interface
bool
catch26_interface__srv__UrosF7Param_Request__Sequence__init(catch26_interface__srv__UrosF7Param_Request__Sequence * array, size_t size);

/// Finalize array of srv/UrosF7Param messages.
/**
 * It calls
 * catch26_interface__srv__UrosF7Param_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_catch26_interface
void
catch26_interface__srv__UrosF7Param_Request__Sequence__fini(catch26_interface__srv__UrosF7Param_Request__Sequence * array);

/// Create array of srv/UrosF7Param messages.
/**
 * It allocates the memory for the array and calls
 * catch26_interface__srv__UrosF7Param_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_catch26_interface
catch26_interface__srv__UrosF7Param_Request__Sequence *
catch26_interface__srv__UrosF7Param_Request__Sequence__create(size_t size);

/// Destroy array of srv/UrosF7Param messages.
/**
 * It calls
 * catch26_interface__srv__UrosF7Param_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_catch26_interface
void
catch26_interface__srv__UrosF7Param_Request__Sequence__destroy(catch26_interface__srv__UrosF7Param_Request__Sequence * array);

/// Check for srv/UrosF7Param message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_catch26_interface
bool
catch26_interface__srv__UrosF7Param_Request__Sequence__are_equal(const catch26_interface__srv__UrosF7Param_Request__Sequence * lhs, const catch26_interface__srv__UrosF7Param_Request__Sequence * rhs);

/// Copy an array of srv/UrosF7Param messages.
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
catch26_interface__srv__UrosF7Param_Request__Sequence__copy(
  const catch26_interface__srv__UrosF7Param_Request__Sequence * input,
  catch26_interface__srv__UrosF7Param_Request__Sequence * output);

/// Initialize srv/UrosF7Param message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * catch26_interface__srv__UrosF7Param_Response
 * )) before or use
 * catch26_interface__srv__UrosF7Param_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_catch26_interface
bool
catch26_interface__srv__UrosF7Param_Response__init(catch26_interface__srv__UrosF7Param_Response * msg);

/// Finalize srv/UrosF7Param message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_catch26_interface
void
catch26_interface__srv__UrosF7Param_Response__fini(catch26_interface__srv__UrosF7Param_Response * msg);

/// Create srv/UrosF7Param message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * catch26_interface__srv__UrosF7Param_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_catch26_interface
catch26_interface__srv__UrosF7Param_Response *
catch26_interface__srv__UrosF7Param_Response__create();

/// Destroy srv/UrosF7Param message.
/**
 * It calls
 * catch26_interface__srv__UrosF7Param_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_catch26_interface
void
catch26_interface__srv__UrosF7Param_Response__destroy(catch26_interface__srv__UrosF7Param_Response * msg);

/// Check for srv/UrosF7Param message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_catch26_interface
bool
catch26_interface__srv__UrosF7Param_Response__are_equal(const catch26_interface__srv__UrosF7Param_Response * lhs, const catch26_interface__srv__UrosF7Param_Response * rhs);

/// Copy a srv/UrosF7Param message.
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
catch26_interface__srv__UrosF7Param_Response__copy(
  const catch26_interface__srv__UrosF7Param_Response * input,
  catch26_interface__srv__UrosF7Param_Response * output);

/// Initialize array of srv/UrosF7Param messages.
/**
 * It allocates the memory for the number of elements and calls
 * catch26_interface__srv__UrosF7Param_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_catch26_interface
bool
catch26_interface__srv__UrosF7Param_Response__Sequence__init(catch26_interface__srv__UrosF7Param_Response__Sequence * array, size_t size);

/// Finalize array of srv/UrosF7Param messages.
/**
 * It calls
 * catch26_interface__srv__UrosF7Param_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_catch26_interface
void
catch26_interface__srv__UrosF7Param_Response__Sequence__fini(catch26_interface__srv__UrosF7Param_Response__Sequence * array);

/// Create array of srv/UrosF7Param messages.
/**
 * It allocates the memory for the array and calls
 * catch26_interface__srv__UrosF7Param_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_catch26_interface
catch26_interface__srv__UrosF7Param_Response__Sequence *
catch26_interface__srv__UrosF7Param_Response__Sequence__create(size_t size);

/// Destroy array of srv/UrosF7Param messages.
/**
 * It calls
 * catch26_interface__srv__UrosF7Param_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_catch26_interface
void
catch26_interface__srv__UrosF7Param_Response__Sequence__destroy(catch26_interface__srv__UrosF7Param_Response__Sequence * array);

/// Check for srv/UrosF7Param message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_catch26_interface
bool
catch26_interface__srv__UrosF7Param_Response__Sequence__are_equal(const catch26_interface__srv__UrosF7Param_Response__Sequence * lhs, const catch26_interface__srv__UrosF7Param_Response__Sequence * rhs);

/// Copy an array of srv/UrosF7Param messages.
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
catch26_interface__srv__UrosF7Param_Response__Sequence__copy(
  const catch26_interface__srv__UrosF7Param_Response__Sequence * input,
  catch26_interface__srv__UrosF7Param_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // CATCH26_INTERFACE__SRV__DETAIL__UROS_F7_PARAM__FUNCTIONS_H_

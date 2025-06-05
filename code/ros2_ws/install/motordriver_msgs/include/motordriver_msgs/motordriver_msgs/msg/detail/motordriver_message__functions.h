// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from motordriver_msgs:msg/MotordriverMessage.idl
// generated code does not contain a copyright notice

#ifndef MOTORDRIVER_MSGS__MSG__DETAIL__MOTORDRIVER_MESSAGE__FUNCTIONS_H_
#define MOTORDRIVER_MSGS__MSG__DETAIL__MOTORDRIVER_MESSAGE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "motordriver_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "motordriver_msgs/msg/detail/motordriver_message__struct.h"

/// Initialize msg/MotordriverMessage message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * motordriver_msgs__msg__MotordriverMessage
 * )) before or use
 * motordriver_msgs__msg__MotordriverMessage__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_motordriver_msgs
bool
motordriver_msgs__msg__MotordriverMessage__init(motordriver_msgs__msg__MotordriverMessage * msg);

/// Finalize msg/MotordriverMessage message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motordriver_msgs
void
motordriver_msgs__msg__MotordriverMessage__fini(motordriver_msgs__msg__MotordriverMessage * msg);

/// Create msg/MotordriverMessage message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * motordriver_msgs__msg__MotordriverMessage__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_motordriver_msgs
motordriver_msgs__msg__MotordriverMessage *
motordriver_msgs__msg__MotordriverMessage__create();

/// Destroy msg/MotordriverMessage message.
/**
 * It calls
 * motordriver_msgs__msg__MotordriverMessage__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motordriver_msgs
void
motordriver_msgs__msg__MotordriverMessage__destroy(motordriver_msgs__msg__MotordriverMessage * msg);

/// Check for msg/MotordriverMessage message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_motordriver_msgs
bool
motordriver_msgs__msg__MotordriverMessage__are_equal(const motordriver_msgs__msg__MotordriverMessage * lhs, const motordriver_msgs__msg__MotordriverMessage * rhs);

/// Copy a msg/MotordriverMessage message.
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
ROSIDL_GENERATOR_C_PUBLIC_motordriver_msgs
bool
motordriver_msgs__msg__MotordriverMessage__copy(
  const motordriver_msgs__msg__MotordriverMessage * input,
  motordriver_msgs__msg__MotordriverMessage * output);

/// Initialize array of msg/MotordriverMessage messages.
/**
 * It allocates the memory for the number of elements and calls
 * motordriver_msgs__msg__MotordriverMessage__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_motordriver_msgs
bool
motordriver_msgs__msg__MotordriverMessage__Sequence__init(motordriver_msgs__msg__MotordriverMessage__Sequence * array, size_t size);

/// Finalize array of msg/MotordriverMessage messages.
/**
 * It calls
 * motordriver_msgs__msg__MotordriverMessage__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motordriver_msgs
void
motordriver_msgs__msg__MotordriverMessage__Sequence__fini(motordriver_msgs__msg__MotordriverMessage__Sequence * array);

/// Create array of msg/MotordriverMessage messages.
/**
 * It allocates the memory for the array and calls
 * motordriver_msgs__msg__MotordriverMessage__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_motordriver_msgs
motordriver_msgs__msg__MotordriverMessage__Sequence *
motordriver_msgs__msg__MotordriverMessage__Sequence__create(size_t size);

/// Destroy array of msg/MotordriverMessage messages.
/**
 * It calls
 * motordriver_msgs__msg__MotordriverMessage__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motordriver_msgs
void
motordriver_msgs__msg__MotordriverMessage__Sequence__destroy(motordriver_msgs__msg__MotordriverMessage__Sequence * array);

/// Check for msg/MotordriverMessage message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_motordriver_msgs
bool
motordriver_msgs__msg__MotordriverMessage__Sequence__are_equal(const motordriver_msgs__msg__MotordriverMessage__Sequence * lhs, const motordriver_msgs__msg__MotordriverMessage__Sequence * rhs);

/// Copy an array of msg/MotordriverMessage messages.
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
ROSIDL_GENERATOR_C_PUBLIC_motordriver_msgs
bool
motordriver_msgs__msg__MotordriverMessage__Sequence__copy(
  const motordriver_msgs__msg__MotordriverMessage__Sequence * input,
  motordriver_msgs__msg__MotordriverMessage__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MOTORDRIVER_MSGS__MSG__DETAIL__MOTORDRIVER_MESSAGE__FUNCTIONS_H_

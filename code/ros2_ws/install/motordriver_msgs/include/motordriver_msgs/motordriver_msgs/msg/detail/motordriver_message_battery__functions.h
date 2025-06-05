// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from motordriver_msgs:msg/MotordriverMessageBattery.idl
// generated code does not contain a copyright notice

#ifndef MOTORDRIVER_MSGS__MSG__DETAIL__MOTORDRIVER_MESSAGE_BATTERY__FUNCTIONS_H_
#define MOTORDRIVER_MSGS__MSG__DETAIL__MOTORDRIVER_MESSAGE_BATTERY__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "motordriver_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "motordriver_msgs/msg/detail/motordriver_message_battery__struct.h"

/// Initialize msg/MotordriverMessageBattery message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * motordriver_msgs__msg__MotordriverMessageBattery
 * )) before or use
 * motordriver_msgs__msg__MotordriverMessageBattery__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_motordriver_msgs
bool
motordriver_msgs__msg__MotordriverMessageBattery__init(motordriver_msgs__msg__MotordriverMessageBattery * msg);

/// Finalize msg/MotordriverMessageBattery message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motordriver_msgs
void
motordriver_msgs__msg__MotordriverMessageBattery__fini(motordriver_msgs__msg__MotordriverMessageBattery * msg);

/// Create msg/MotordriverMessageBattery message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * motordriver_msgs__msg__MotordriverMessageBattery__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_motordriver_msgs
motordriver_msgs__msg__MotordriverMessageBattery *
motordriver_msgs__msg__MotordriverMessageBattery__create();

/// Destroy msg/MotordriverMessageBattery message.
/**
 * It calls
 * motordriver_msgs__msg__MotordriverMessageBattery__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motordriver_msgs
void
motordriver_msgs__msg__MotordriverMessageBattery__destroy(motordriver_msgs__msg__MotordriverMessageBattery * msg);

/// Check for msg/MotordriverMessageBattery message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_motordriver_msgs
bool
motordriver_msgs__msg__MotordriverMessageBattery__are_equal(const motordriver_msgs__msg__MotordriverMessageBattery * lhs, const motordriver_msgs__msg__MotordriverMessageBattery * rhs);

/// Copy a msg/MotordriverMessageBattery message.
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
motordriver_msgs__msg__MotordriverMessageBattery__copy(
  const motordriver_msgs__msg__MotordriverMessageBattery * input,
  motordriver_msgs__msg__MotordriverMessageBattery * output);

/// Initialize array of msg/MotordriverMessageBattery messages.
/**
 * It allocates the memory for the number of elements and calls
 * motordriver_msgs__msg__MotordriverMessageBattery__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_motordriver_msgs
bool
motordriver_msgs__msg__MotordriverMessageBattery__Sequence__init(motordriver_msgs__msg__MotordriverMessageBattery__Sequence * array, size_t size);

/// Finalize array of msg/MotordriverMessageBattery messages.
/**
 * It calls
 * motordriver_msgs__msg__MotordriverMessageBattery__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motordriver_msgs
void
motordriver_msgs__msg__MotordriverMessageBattery__Sequence__fini(motordriver_msgs__msg__MotordriverMessageBattery__Sequence * array);

/// Create array of msg/MotordriverMessageBattery messages.
/**
 * It allocates the memory for the array and calls
 * motordriver_msgs__msg__MotordriverMessageBattery__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_motordriver_msgs
motordriver_msgs__msg__MotordriverMessageBattery__Sequence *
motordriver_msgs__msg__MotordriverMessageBattery__Sequence__create(size_t size);

/// Destroy array of msg/MotordriverMessageBattery messages.
/**
 * It calls
 * motordriver_msgs__msg__MotordriverMessageBattery__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motordriver_msgs
void
motordriver_msgs__msg__MotordriverMessageBattery__Sequence__destroy(motordriver_msgs__msg__MotordriverMessageBattery__Sequence * array);

/// Check for msg/MotordriverMessageBattery message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_motordriver_msgs
bool
motordriver_msgs__msg__MotordriverMessageBattery__Sequence__are_equal(const motordriver_msgs__msg__MotordriverMessageBattery__Sequence * lhs, const motordriver_msgs__msg__MotordriverMessageBattery__Sequence * rhs);

/// Copy an array of msg/MotordriverMessageBattery messages.
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
motordriver_msgs__msg__MotordriverMessageBattery__Sequence__copy(
  const motordriver_msgs__msg__MotordriverMessageBattery__Sequence * input,
  motordriver_msgs__msg__MotordriverMessageBattery__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MOTORDRIVER_MSGS__MSG__DETAIL__MOTORDRIVER_MESSAGE_BATTERY__FUNCTIONS_H_

// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from motordriver_msgs:msg/MotordriverMessage.idl
// generated code does not contain a copyright notice

#ifndef MOTORDRIVER_MSGS__MSG__DETAIL__MOTORDRIVER_MESSAGE__STRUCT_H_
#define MOTORDRIVER_MSGS__MSG__DETAIL__MOTORDRIVER_MESSAGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/MotordriverMessage in the package motordriver_msgs.
typedef struct motordriver_msgs__msg__MotordriverMessage
{
  int32_t encoder1;
  int32_t encoder2;
  int32_t speed1;
  int32_t speed2;
  int32_t pwm1;
  int32_t pwm2;
} motordriver_msgs__msg__MotordriverMessage;

// Struct for a sequence of motordriver_msgs__msg__MotordriverMessage.
typedef struct motordriver_msgs__msg__MotordriverMessage__Sequence
{
  motordriver_msgs__msg__MotordriverMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motordriver_msgs__msg__MotordriverMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOTORDRIVER_MSGS__MSG__DETAIL__MOTORDRIVER_MESSAGE__STRUCT_H_

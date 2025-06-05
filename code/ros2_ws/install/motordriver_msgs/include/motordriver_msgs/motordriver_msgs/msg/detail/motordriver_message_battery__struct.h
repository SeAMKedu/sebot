// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from motordriver_msgs:msg/MotordriverMessageBattery.idl
// generated code does not contain a copyright notice

#ifndef MOTORDRIVER_MSGS__MSG__DETAIL__MOTORDRIVER_MESSAGE_BATTERY__STRUCT_H_
#define MOTORDRIVER_MSGS__MSG__DETAIL__MOTORDRIVER_MESSAGE_BATTERY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/MotordriverMessageBattery in the package motordriver_msgs.
typedef struct motordriver_msgs__msg__MotordriverMessageBattery
{
  int32_t encoder1;
  int32_t encoder2;
  int32_t speed1;
  int32_t speed2;
  int32_t pwm1;
  int32_t pwm2;
  float battery;
} motordriver_msgs__msg__MotordriverMessageBattery;

// Struct for a sequence of motordriver_msgs__msg__MotordriverMessageBattery.
typedef struct motordriver_msgs__msg__MotordriverMessageBattery__Sequence
{
  motordriver_msgs__msg__MotordriverMessageBattery * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motordriver_msgs__msg__MotordriverMessageBattery__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOTORDRIVER_MSGS__MSG__DETAIL__MOTORDRIVER_MESSAGE_BATTERY__STRUCT_H_

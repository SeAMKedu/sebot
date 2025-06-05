// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from motordriver_msgs:msg/MotordriverMessage.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "motordriver_msgs/msg/detail/motordriver_message__rosidl_typesupport_introspection_c.h"
#include "motordriver_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "motordriver_msgs/msg/detail/motordriver_message__functions.h"
#include "motordriver_msgs/msg/detail/motordriver_message__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void motordriver_msgs__msg__MotordriverMessage__rosidl_typesupport_introspection_c__MotordriverMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  motordriver_msgs__msg__MotordriverMessage__init(message_memory);
}

void motordriver_msgs__msg__MotordriverMessage__rosidl_typesupport_introspection_c__MotordriverMessage_fini_function(void * message_memory)
{
  motordriver_msgs__msg__MotordriverMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember motordriver_msgs__msg__MotordriverMessage__rosidl_typesupport_introspection_c__MotordriverMessage_message_member_array[6] = {
  {
    "encoder1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motordriver_msgs__msg__MotordriverMessage, encoder1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "encoder2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motordriver_msgs__msg__MotordriverMessage, encoder2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "speed1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motordriver_msgs__msg__MotordriverMessage, speed1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "speed2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motordriver_msgs__msg__MotordriverMessage, speed2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pwm1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motordriver_msgs__msg__MotordriverMessage, pwm1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pwm2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motordriver_msgs__msg__MotordriverMessage, pwm2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers motordriver_msgs__msg__MotordriverMessage__rosidl_typesupport_introspection_c__MotordriverMessage_message_members = {
  "motordriver_msgs__msg",  // message namespace
  "MotordriverMessage",  // message name
  6,  // number of fields
  sizeof(motordriver_msgs__msg__MotordriverMessage),
  motordriver_msgs__msg__MotordriverMessage__rosidl_typesupport_introspection_c__MotordriverMessage_message_member_array,  // message members
  motordriver_msgs__msg__MotordriverMessage__rosidl_typesupport_introspection_c__MotordriverMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  motordriver_msgs__msg__MotordriverMessage__rosidl_typesupport_introspection_c__MotordriverMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t motordriver_msgs__msg__MotordriverMessage__rosidl_typesupport_introspection_c__MotordriverMessage_message_type_support_handle = {
  0,
  &motordriver_msgs__msg__MotordriverMessage__rosidl_typesupport_introspection_c__MotordriverMessage_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_motordriver_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, motordriver_msgs, msg, MotordriverMessage)() {
  if (!motordriver_msgs__msg__MotordriverMessage__rosidl_typesupport_introspection_c__MotordriverMessage_message_type_support_handle.typesupport_identifier) {
    motordriver_msgs__msg__MotordriverMessage__rosidl_typesupport_introspection_c__MotordriverMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &motordriver_msgs__msg__MotordriverMessage__rosidl_typesupport_introspection_c__MotordriverMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

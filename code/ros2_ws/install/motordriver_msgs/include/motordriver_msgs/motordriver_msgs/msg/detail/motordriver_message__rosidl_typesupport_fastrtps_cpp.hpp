// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from motordriver_msgs:msg/MotordriverMessage.idl
// generated code does not contain a copyright notice

#ifndef MOTORDRIVER_MSGS__MSG__DETAIL__MOTORDRIVER_MESSAGE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define MOTORDRIVER_MSGS__MSG__DETAIL__MOTORDRIVER_MESSAGE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "motordriver_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "motordriver_msgs/msg/detail/motordriver_message__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace motordriver_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_motordriver_msgs
cdr_serialize(
  const motordriver_msgs::msg::MotordriverMessage & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_motordriver_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  motordriver_msgs::msg::MotordriverMessage & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_motordriver_msgs
get_serialized_size(
  const motordriver_msgs::msg::MotordriverMessage & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_motordriver_msgs
max_serialized_size_MotordriverMessage(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace motordriver_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_motordriver_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, motordriver_msgs, msg, MotordriverMessage)();

#ifdef __cplusplus
}
#endif

#endif  // MOTORDRIVER_MSGS__MSG__DETAIL__MOTORDRIVER_MESSAGE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

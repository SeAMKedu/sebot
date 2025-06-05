// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from motordriver_msgs:msg/MotordriverMessage.idl
// generated code does not contain a copyright notice

#ifndef MOTORDRIVER_MSGS__MSG__DETAIL__MOTORDRIVER_MESSAGE__BUILDER_HPP_
#define MOTORDRIVER_MSGS__MSG__DETAIL__MOTORDRIVER_MESSAGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "motordriver_msgs/msg/detail/motordriver_message__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace motordriver_msgs
{

namespace msg
{

namespace builder
{

class Init_MotordriverMessage_pwm2
{
public:
  explicit Init_MotordriverMessage_pwm2(::motordriver_msgs::msg::MotordriverMessage & msg)
  : msg_(msg)
  {}
  ::motordriver_msgs::msg::MotordriverMessage pwm2(::motordriver_msgs::msg::MotordriverMessage::_pwm2_type arg)
  {
    msg_.pwm2 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motordriver_msgs::msg::MotordriverMessage msg_;
};

class Init_MotordriverMessage_pwm1
{
public:
  explicit Init_MotordriverMessage_pwm1(::motordriver_msgs::msg::MotordriverMessage & msg)
  : msg_(msg)
  {}
  Init_MotordriverMessage_pwm2 pwm1(::motordriver_msgs::msg::MotordriverMessage::_pwm1_type arg)
  {
    msg_.pwm1 = std::move(arg);
    return Init_MotordriverMessage_pwm2(msg_);
  }

private:
  ::motordriver_msgs::msg::MotordriverMessage msg_;
};

class Init_MotordriverMessage_speed2
{
public:
  explicit Init_MotordriverMessage_speed2(::motordriver_msgs::msg::MotordriverMessage & msg)
  : msg_(msg)
  {}
  Init_MotordriverMessage_pwm1 speed2(::motordriver_msgs::msg::MotordriverMessage::_speed2_type arg)
  {
    msg_.speed2 = std::move(arg);
    return Init_MotordriverMessage_pwm1(msg_);
  }

private:
  ::motordriver_msgs::msg::MotordriverMessage msg_;
};

class Init_MotordriverMessage_speed1
{
public:
  explicit Init_MotordriverMessage_speed1(::motordriver_msgs::msg::MotordriverMessage & msg)
  : msg_(msg)
  {}
  Init_MotordriverMessage_speed2 speed1(::motordriver_msgs::msg::MotordriverMessage::_speed1_type arg)
  {
    msg_.speed1 = std::move(arg);
    return Init_MotordriverMessage_speed2(msg_);
  }

private:
  ::motordriver_msgs::msg::MotordriverMessage msg_;
};

class Init_MotordriverMessage_encoder2
{
public:
  explicit Init_MotordriverMessage_encoder2(::motordriver_msgs::msg::MotordriverMessage & msg)
  : msg_(msg)
  {}
  Init_MotordriverMessage_speed1 encoder2(::motordriver_msgs::msg::MotordriverMessage::_encoder2_type arg)
  {
    msg_.encoder2 = std::move(arg);
    return Init_MotordriverMessage_speed1(msg_);
  }

private:
  ::motordriver_msgs::msg::MotordriverMessage msg_;
};

class Init_MotordriverMessage_encoder1
{
public:
  Init_MotordriverMessage_encoder1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotordriverMessage_encoder2 encoder1(::motordriver_msgs::msg::MotordriverMessage::_encoder1_type arg)
  {
    msg_.encoder1 = std::move(arg);
    return Init_MotordriverMessage_encoder2(msg_);
  }

private:
  ::motordriver_msgs::msg::MotordriverMessage msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::motordriver_msgs::msg::MotordriverMessage>()
{
  return motordriver_msgs::msg::builder::Init_MotordriverMessage_encoder1();
}

}  // namespace motordriver_msgs

#endif  // MOTORDRIVER_MSGS__MSG__DETAIL__MOTORDRIVER_MESSAGE__BUILDER_HPP_

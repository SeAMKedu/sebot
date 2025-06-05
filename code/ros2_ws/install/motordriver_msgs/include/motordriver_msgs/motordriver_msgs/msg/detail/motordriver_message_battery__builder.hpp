// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from motordriver_msgs:msg/MotordriverMessageBattery.idl
// generated code does not contain a copyright notice

#ifndef MOTORDRIVER_MSGS__MSG__DETAIL__MOTORDRIVER_MESSAGE_BATTERY__BUILDER_HPP_
#define MOTORDRIVER_MSGS__MSG__DETAIL__MOTORDRIVER_MESSAGE_BATTERY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "motordriver_msgs/msg/detail/motordriver_message_battery__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace motordriver_msgs
{

namespace msg
{

namespace builder
{

class Init_MotordriverMessageBattery_battery
{
public:
  explicit Init_MotordriverMessageBattery_battery(::motordriver_msgs::msg::MotordriverMessageBattery & msg)
  : msg_(msg)
  {}
  ::motordriver_msgs::msg::MotordriverMessageBattery battery(::motordriver_msgs::msg::MotordriverMessageBattery::_battery_type arg)
  {
    msg_.battery = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motordriver_msgs::msg::MotordriverMessageBattery msg_;
};

class Init_MotordriverMessageBattery_pwm2
{
public:
  explicit Init_MotordriverMessageBattery_pwm2(::motordriver_msgs::msg::MotordriverMessageBattery & msg)
  : msg_(msg)
  {}
  Init_MotordriverMessageBattery_battery pwm2(::motordriver_msgs::msg::MotordriverMessageBattery::_pwm2_type arg)
  {
    msg_.pwm2 = std::move(arg);
    return Init_MotordriverMessageBattery_battery(msg_);
  }

private:
  ::motordriver_msgs::msg::MotordriverMessageBattery msg_;
};

class Init_MotordriverMessageBattery_pwm1
{
public:
  explicit Init_MotordriverMessageBattery_pwm1(::motordriver_msgs::msg::MotordriverMessageBattery & msg)
  : msg_(msg)
  {}
  Init_MotordriverMessageBattery_pwm2 pwm1(::motordriver_msgs::msg::MotordriverMessageBattery::_pwm1_type arg)
  {
    msg_.pwm1 = std::move(arg);
    return Init_MotordriverMessageBattery_pwm2(msg_);
  }

private:
  ::motordriver_msgs::msg::MotordriverMessageBattery msg_;
};

class Init_MotordriverMessageBattery_speed2
{
public:
  explicit Init_MotordriverMessageBattery_speed2(::motordriver_msgs::msg::MotordriverMessageBattery & msg)
  : msg_(msg)
  {}
  Init_MotordriverMessageBattery_pwm1 speed2(::motordriver_msgs::msg::MotordriverMessageBattery::_speed2_type arg)
  {
    msg_.speed2 = std::move(arg);
    return Init_MotordriverMessageBattery_pwm1(msg_);
  }

private:
  ::motordriver_msgs::msg::MotordriverMessageBattery msg_;
};

class Init_MotordriverMessageBattery_speed1
{
public:
  explicit Init_MotordriverMessageBattery_speed1(::motordriver_msgs::msg::MotordriverMessageBattery & msg)
  : msg_(msg)
  {}
  Init_MotordriverMessageBattery_speed2 speed1(::motordriver_msgs::msg::MotordriverMessageBattery::_speed1_type arg)
  {
    msg_.speed1 = std::move(arg);
    return Init_MotordriverMessageBattery_speed2(msg_);
  }

private:
  ::motordriver_msgs::msg::MotordriverMessageBattery msg_;
};

class Init_MotordriverMessageBattery_encoder2
{
public:
  explicit Init_MotordriverMessageBattery_encoder2(::motordriver_msgs::msg::MotordriverMessageBattery & msg)
  : msg_(msg)
  {}
  Init_MotordriverMessageBattery_speed1 encoder2(::motordriver_msgs::msg::MotordriverMessageBattery::_encoder2_type arg)
  {
    msg_.encoder2 = std::move(arg);
    return Init_MotordriverMessageBattery_speed1(msg_);
  }

private:
  ::motordriver_msgs::msg::MotordriverMessageBattery msg_;
};

class Init_MotordriverMessageBattery_encoder1
{
public:
  Init_MotordriverMessageBattery_encoder1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotordriverMessageBattery_encoder2 encoder1(::motordriver_msgs::msg::MotordriverMessageBattery::_encoder1_type arg)
  {
    msg_.encoder1 = std::move(arg);
    return Init_MotordriverMessageBattery_encoder2(msg_);
  }

private:
  ::motordriver_msgs::msg::MotordriverMessageBattery msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::motordriver_msgs::msg::MotordriverMessageBattery>()
{
  return motordriver_msgs::msg::builder::Init_MotordriverMessageBattery_encoder1();
}

}  // namespace motordriver_msgs

#endif  // MOTORDRIVER_MSGS__MSG__DETAIL__MOTORDRIVER_MESSAGE_BATTERY__BUILDER_HPP_

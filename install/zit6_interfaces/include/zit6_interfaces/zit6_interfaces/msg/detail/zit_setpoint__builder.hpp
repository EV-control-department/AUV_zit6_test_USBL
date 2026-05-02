// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from zit6_interfaces:msg/ZitSetpoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "zit6_interfaces/msg/zit_setpoint.hpp"


#ifndef ZIT6_INTERFACES__MSG__DETAIL__ZIT_SETPOINT__BUILDER_HPP_
#define ZIT6_INTERFACES__MSG__DETAIL__ZIT_SETPOINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "zit6_interfaces/msg/detail/zit_setpoint__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace zit6_interfaces
{

namespace msg
{

namespace builder
{

class Init_ZitSetpoint_seq
{
public:
  explicit Init_ZitSetpoint_seq(::zit6_interfaces::msg::ZitSetpoint & msg)
  : msg_(msg)
  {}
  ::zit6_interfaces::msg::ZitSetpoint seq(::zit6_interfaces::msg::ZitSetpoint::_seq_type arg)
  {
    msg_.seq = std::move(arg);
    return std::move(msg_);
  }

private:
  ::zit6_interfaces::msg::ZitSetpoint msg_;
};

class Init_ZitSetpoint_yaw
{
public:
  explicit Init_ZitSetpoint_yaw(::zit6_interfaces::msg::ZitSetpoint & msg)
  : msg_(msg)
  {}
  Init_ZitSetpoint_seq yaw(::zit6_interfaces::msg::ZitSetpoint::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_ZitSetpoint_seq(msg_);
  }

private:
  ::zit6_interfaces::msg::ZitSetpoint msg_;
};

class Init_ZitSetpoint_z
{
public:
  explicit Init_ZitSetpoint_z(::zit6_interfaces::msg::ZitSetpoint & msg)
  : msg_(msg)
  {}
  Init_ZitSetpoint_yaw z(::zit6_interfaces::msg::ZitSetpoint::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_ZitSetpoint_yaw(msg_);
  }

private:
  ::zit6_interfaces::msg::ZitSetpoint msg_;
};

class Init_ZitSetpoint_y
{
public:
  explicit Init_ZitSetpoint_y(::zit6_interfaces::msg::ZitSetpoint & msg)
  : msg_(msg)
  {}
  Init_ZitSetpoint_z y(::zit6_interfaces::msg::ZitSetpoint::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_ZitSetpoint_z(msg_);
  }

private:
  ::zit6_interfaces::msg::ZitSetpoint msg_;
};

class Init_ZitSetpoint_x
{
public:
  explicit Init_ZitSetpoint_x(::zit6_interfaces::msg::ZitSetpoint & msg)
  : msg_(msg)
  {}
  Init_ZitSetpoint_y x(::zit6_interfaces::msg::ZitSetpoint::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_ZitSetpoint_y(msg_);
  }

private:
  ::zit6_interfaces::msg::ZitSetpoint msg_;
};

class Init_ZitSetpoint_type_mask
{
public:
  explicit Init_ZitSetpoint_type_mask(::zit6_interfaces::msg::ZitSetpoint & msg)
  : msg_(msg)
  {}
  Init_ZitSetpoint_x type_mask(::zit6_interfaces::msg::ZitSetpoint::_type_mask_type arg)
  {
    msg_.type_mask = std::move(arg);
    return Init_ZitSetpoint_x(msg_);
  }

private:
  ::zit6_interfaces::msg::ZitSetpoint msg_;
};

class Init_ZitSetpoint_control_key
{
public:
  Init_ZitSetpoint_control_key()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ZitSetpoint_type_mask control_key(::zit6_interfaces::msg::ZitSetpoint::_control_key_type arg)
  {
    msg_.control_key = std::move(arg);
    return Init_ZitSetpoint_type_mask(msg_);
  }

private:
  ::zit6_interfaces::msg::ZitSetpoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::zit6_interfaces::msg::ZitSetpoint>()
{
  return zit6_interfaces::msg::builder::Init_ZitSetpoint_control_key();
}

}  // namespace zit6_interfaces

#endif  // ZIT6_INTERFACES__MSG__DETAIL__ZIT_SETPOINT__BUILDER_HPP_

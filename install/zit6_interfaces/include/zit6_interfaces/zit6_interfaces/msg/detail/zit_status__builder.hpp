// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from zit6_interfaces:msg/ZitStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "zit6_interfaces/msg/zit_status.hpp"


#ifndef ZIT6_INTERFACES__MSG__DETAIL__ZIT_STATUS__BUILDER_HPP_
#define ZIT6_INTERFACES__MSG__DETAIL__ZIT_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "zit6_interfaces/msg/detail/zit_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace zit6_interfaces
{

namespace msg
{

namespace builder
{

class Init_ZitStatus_error_flags
{
public:
  explicit Init_ZitStatus_error_flags(::zit6_interfaces::msg::ZitStatus & msg)
  : msg_(msg)
  {}
  ::zit6_interfaces::msg::ZitStatus error_flags(::zit6_interfaces::msg::ZitStatus::_error_flags_type arg)
  {
    msg_.error_flags = std::move(arg);
    return std::move(msg_);
  }

private:
  ::zit6_interfaces::msg::ZitStatus msg_;
};

class Init_ZitStatus_battery_voltage
{
public:
  explicit Init_ZitStatus_battery_voltage(::zit6_interfaces::msg::ZitStatus & msg)
  : msg_(msg)
  {}
  Init_ZitStatus_error_flags battery_voltage(::zit6_interfaces::msg::ZitStatus::_battery_voltage_type arg)
  {
    msg_.battery_voltage = std::move(arg);
    return Init_ZitStatus_error_flags(msg_);
  }

private:
  ::zit6_interfaces::msg::ZitStatus msg_;
};

class Init_ZitStatus_cycle_time_ms
{
public:
  explicit Init_ZitStatus_cycle_time_ms(::zit6_interfaces::msg::ZitStatus & msg)
  : msg_(msg)
  {}
  Init_ZitStatus_battery_voltage cycle_time_ms(::zit6_interfaces::msg::ZitStatus::_cycle_time_ms_type arg)
  {
    msg_.cycle_time_ms = std::move(arg);
    return Init_ZitStatus_battery_voltage(msg_);
  }

private:
  ::zit6_interfaces::msg::ZitStatus msg_;
};

class Init_ZitStatus_forces
{
public:
  explicit Init_ZitStatus_forces(::zit6_interfaces::msg::ZitStatus & msg)
  : msg_(msg)
  {}
  Init_ZitStatus_cycle_time_ms forces(::zit6_interfaces::msg::ZitStatus::_forces_type arg)
  {
    msg_.forces = std::move(arg);
    return Init_ZitStatus_cycle_time_ms(msg_);
  }

private:
  ::zit6_interfaces::msg::ZitStatus msg_;
};

class Init_ZitStatus_navigation_ready
{
public:
  explicit Init_ZitStatus_navigation_ready(::zit6_interfaces::msg::ZitStatus & msg)
  : msg_(msg)
  {}
  Init_ZitStatus_forces navigation_ready(::zit6_interfaces::msg::ZitStatus::_navigation_ready_type arg)
  {
    msg_.navigation_ready = std::move(arg);
    return Init_ZitStatus_forces(msg_);
  }

private:
  ::zit6_interfaces::msg::ZitStatus msg_;
};

class Init_ZitStatus_control_level
{
public:
  explicit Init_ZitStatus_control_level(::zit6_interfaces::msg::ZitStatus & msg)
  : msg_(msg)
  {}
  Init_ZitStatus_navigation_ready control_level(::zit6_interfaces::msg::ZitStatus::_control_level_type arg)
  {
    msg_.control_level = std::move(arg);
    return Init_ZitStatus_navigation_ready(msg_);
  }

private:
  ::zit6_interfaces::msg::ZitStatus msg_;
};

class Init_ZitStatus_is_armed
{
public:
  Init_ZitStatus_is_armed()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ZitStatus_control_level is_armed(::zit6_interfaces::msg::ZitStatus::_is_armed_type arg)
  {
    msg_.is_armed = std::move(arg);
    return Init_ZitStatus_control_level(msg_);
  }

private:
  ::zit6_interfaces::msg::ZitStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::zit6_interfaces::msg::ZitStatus>()
{
  return zit6_interfaces::msg::builder::Init_ZitStatus_is_armed();
}

}  // namespace zit6_interfaces

#endif  // ZIT6_INTERFACES__MSG__DETAIL__ZIT_STATUS__BUILDER_HPP_

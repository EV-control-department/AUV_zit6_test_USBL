// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from zit6_interfaces:msg/ZitStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "zit6_interfaces/msg/zit_status.hpp"


#ifndef ZIT6_INTERFACES__MSG__DETAIL__ZIT_STATUS__TRAITS_HPP_
#define ZIT6_INTERFACES__MSG__DETAIL__ZIT_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "zit6_interfaces/msg/detail/zit_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace zit6_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ZitStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: is_armed
  {
    out << "is_armed: ";
    rosidl_generator_traits::value_to_yaml(msg.is_armed, out);
    out << ", ";
  }

  // member: control_level
  {
    out << "control_level: ";
    rosidl_generator_traits::value_to_yaml(msg.control_level, out);
    out << ", ";
  }

  // member: navigation_ready
  {
    out << "navigation_ready: ";
    rosidl_generator_traits::value_to_yaml(msg.navigation_ready, out);
    out << ", ";
  }

  // member: forces
  {
    if (msg.forces.size() == 0) {
      out << "forces: []";
    } else {
      out << "forces: [";
      size_t pending_items = msg.forces.size();
      for (auto item : msg.forces) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: cycle_time_ms
  {
    out << "cycle_time_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.cycle_time_ms, out);
    out << ", ";
  }

  // member: battery_voltage
  {
    out << "battery_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_voltage, out);
    out << ", ";
  }

  // member: error_flags
  {
    out << "error_flags: ";
    rosidl_generator_traits::value_to_yaml(msg.error_flags, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ZitStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: is_armed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_armed: ";
    rosidl_generator_traits::value_to_yaml(msg.is_armed, out);
    out << "\n";
  }

  // member: control_level
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "control_level: ";
    rosidl_generator_traits::value_to_yaml(msg.control_level, out);
    out << "\n";
  }

  // member: navigation_ready
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "navigation_ready: ";
    rosidl_generator_traits::value_to_yaml(msg.navigation_ready, out);
    out << "\n";
  }

  // member: forces
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.forces.size() == 0) {
      out << "forces: []\n";
    } else {
      out << "forces:\n";
      for (auto item : msg.forces) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: cycle_time_ms
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cycle_time_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.cycle_time_ms, out);
    out << "\n";
  }

  // member: battery_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_voltage, out);
    out << "\n";
  }

  // member: error_flags
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_flags: ";
    rosidl_generator_traits::value_to_yaml(msg.error_flags, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ZitStatus & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace zit6_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use zit6_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const zit6_interfaces::msg::ZitStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  zit6_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use zit6_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const zit6_interfaces::msg::ZitStatus & msg)
{
  return zit6_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<zit6_interfaces::msg::ZitStatus>()
{
  return "zit6_interfaces::msg::ZitStatus";
}

template<>
inline const char * name<zit6_interfaces::msg::ZitStatus>()
{
  return "zit6_interfaces/msg/ZitStatus";
}

template<>
struct has_fixed_size<zit6_interfaces::msg::ZitStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<zit6_interfaces::msg::ZitStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<zit6_interfaces::msg::ZitStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ZIT6_INTERFACES__MSG__DETAIL__ZIT_STATUS__TRAITS_HPP_

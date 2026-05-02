// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from zit6_interfaces:msg/ZitSetpoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "zit6_interfaces/msg/zit_setpoint.hpp"


#ifndef ZIT6_INTERFACES__MSG__DETAIL__ZIT_SETPOINT__TRAITS_HPP_
#define ZIT6_INTERFACES__MSG__DETAIL__ZIT_SETPOINT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "zit6_interfaces/msg/detail/zit_setpoint__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace zit6_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ZitSetpoint & msg,
  std::ostream & out)
{
  out << "{";
  // member: control_key
  {
    out << "control_key: ";
    rosidl_generator_traits::value_to_yaml(msg.control_key, out);
    out << ", ";
  }

  // member: type_mask
  {
    out << "type_mask: ";
    rosidl_generator_traits::value_to_yaml(msg.type_mask, out);
    out << ", ";
  }

  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: z
  {
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << ", ";
  }

  // member: seq
  {
    out << "seq: ";
    rosidl_generator_traits::value_to_yaml(msg.seq, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ZitSetpoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: control_key
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "control_key: ";
    rosidl_generator_traits::value_to_yaml(msg.control_key, out);
    out << "\n";
  }

  // member: type_mask
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "type_mask: ";
    rosidl_generator_traits::value_to_yaml(msg.type_mask, out);
    out << "\n";
  }

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }

  // member: seq
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "seq: ";
    rosidl_generator_traits::value_to_yaml(msg.seq, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ZitSetpoint & msg, bool use_flow_style = false)
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
  const zit6_interfaces::msg::ZitSetpoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  zit6_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use zit6_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const zit6_interfaces::msg::ZitSetpoint & msg)
{
  return zit6_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<zit6_interfaces::msg::ZitSetpoint>()
{
  return "zit6_interfaces::msg::ZitSetpoint";
}

template<>
inline const char * name<zit6_interfaces::msg::ZitSetpoint>()
{
  return "zit6_interfaces/msg/ZitSetpoint";
}

template<>
struct has_fixed_size<zit6_interfaces::msg::ZitSetpoint>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<zit6_interfaces::msg::ZitSetpoint>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<zit6_interfaces::msg::ZitSetpoint>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ZIT6_INTERFACES__MSG__DETAIL__ZIT_SETPOINT__TRAITS_HPP_

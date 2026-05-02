// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from zit6_interfaces:msg/ZitSetpoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "zit6_interfaces/msg/zit_setpoint.hpp"


#ifndef ZIT6_INTERFACES__MSG__DETAIL__ZIT_SETPOINT__STRUCT_HPP_
#define ZIT6_INTERFACES__MSG__DETAIL__ZIT_SETPOINT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__zit6_interfaces__msg__ZitSetpoint __attribute__((deprecated))
#else
# define DEPRECATED__zit6_interfaces__msg__ZitSetpoint __declspec(deprecated)
#endif

namespace zit6_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ZitSetpoint_
{
  using Type = ZitSetpoint_<ContainerAllocator>;

  explicit ZitSetpoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->control_key = 0;
      this->type_mask = 0;
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
      this->yaw = 0.0f;
      this->seq = 0ul;
    }
  }

  explicit ZitSetpoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->control_key = 0;
      this->type_mask = 0;
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
      this->yaw = 0.0f;
      this->seq = 0ul;
    }
  }

  // field types and members
  using _control_key_type =
    uint8_t;
  _control_key_type control_key;
  using _type_mask_type =
    uint8_t;
  _type_mask_type type_mask;
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;
  using _z_type =
    float;
  _z_type z;
  using _yaw_type =
    float;
  _yaw_type yaw;
  using _seq_type =
    uint32_t;
  _seq_type seq;

  // setters for named parameter idiom
  Type & set__control_key(
    const uint8_t & _arg)
  {
    this->control_key = _arg;
    return *this;
  }
  Type & set__type_mask(
    const uint8_t & _arg)
  {
    this->type_mask = _arg;
    return *this;
  }
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const float & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__yaw(
    const float & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__seq(
    const uint32_t & _arg)
  {
    this->seq = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    zit6_interfaces::msg::ZitSetpoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const zit6_interfaces::msg::ZitSetpoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<zit6_interfaces::msg::ZitSetpoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<zit6_interfaces::msg::ZitSetpoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      zit6_interfaces::msg::ZitSetpoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<zit6_interfaces::msg::ZitSetpoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      zit6_interfaces::msg::ZitSetpoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<zit6_interfaces::msg::ZitSetpoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<zit6_interfaces::msg::ZitSetpoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<zit6_interfaces::msg::ZitSetpoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__zit6_interfaces__msg__ZitSetpoint
    std::shared_ptr<zit6_interfaces::msg::ZitSetpoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__zit6_interfaces__msg__ZitSetpoint
    std::shared_ptr<zit6_interfaces::msg::ZitSetpoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ZitSetpoint_ & other) const
  {
    if (this->control_key != other.control_key) {
      return false;
    }
    if (this->type_mask != other.type_mask) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->seq != other.seq) {
      return false;
    }
    return true;
  }
  bool operator!=(const ZitSetpoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ZitSetpoint_

// alias to use template instance with default allocator
using ZitSetpoint =
  zit6_interfaces::msg::ZitSetpoint_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace zit6_interfaces

#endif  // ZIT6_INTERFACES__MSG__DETAIL__ZIT_SETPOINT__STRUCT_HPP_

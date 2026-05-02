// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from zit6_interfaces:msg/ZitStatus.idl
// generated code does not contain a copyright notice

#ifndef ZIT6_INTERFACES__MSG__DETAIL__ZIT_STATUS__STRUCT_H_
#define ZIT6_INTERFACES__MSG__DETAIL__ZIT_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ZitStatus in the package zit6_interfaces.
typedef struct zit6_interfaces__msg__ZitStatus
{
  bool is_armed;
  uint8_t arm_mode;
  uint8_t control_level;
  uint8_t ins_state;
  bool navigation_ready;
  float forces[4];
  float cycle_time_ms;
  float battery_voltage;
  uint32_t error_flags;
} zit6_interfaces__msg__ZitStatus;

// Struct for a sequence of zit6_interfaces__msg__ZitStatus.
typedef struct zit6_interfaces__msg__ZitStatus__Sequence
{
  zit6_interfaces__msg__ZitStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} zit6_interfaces__msg__ZitStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ZIT6_INTERFACES__MSG__DETAIL__ZIT_STATUS__STRUCT_H_

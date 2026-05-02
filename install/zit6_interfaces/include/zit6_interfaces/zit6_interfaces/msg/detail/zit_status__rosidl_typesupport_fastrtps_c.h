// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from zit6_interfaces:msg/ZitStatus.idl
// generated code does not contain a copyright notice
#ifndef ZIT6_INTERFACES__MSG__DETAIL__ZIT_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define ZIT6_INTERFACES__MSG__DETAIL__ZIT_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "zit6_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "zit6_interfaces/msg/detail/zit_status__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zit6_interfaces
bool cdr_serialize_zit6_interfaces__msg__ZitStatus(
  const zit6_interfaces__msg__ZitStatus * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zit6_interfaces
bool cdr_deserialize_zit6_interfaces__msg__ZitStatus(
  eprosima::fastcdr::Cdr &,
  zit6_interfaces__msg__ZitStatus * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zit6_interfaces
size_t get_serialized_size_zit6_interfaces__msg__ZitStatus(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zit6_interfaces
size_t max_serialized_size_zit6_interfaces__msg__ZitStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zit6_interfaces
bool cdr_serialize_key_zit6_interfaces__msg__ZitStatus(
  const zit6_interfaces__msg__ZitStatus * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zit6_interfaces
size_t get_serialized_size_key_zit6_interfaces__msg__ZitStatus(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zit6_interfaces
size_t max_serialized_size_key_zit6_interfaces__msg__ZitStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zit6_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, zit6_interfaces, msg, ZitStatus)();

#ifdef __cplusplus
}
#endif

#endif  // ZIT6_INTERFACES__MSG__DETAIL__ZIT_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_

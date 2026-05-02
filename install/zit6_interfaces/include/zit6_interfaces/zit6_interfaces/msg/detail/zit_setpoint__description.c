// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from zit6_interfaces:msg/ZitSetpoint.idl
// generated code does not contain a copyright notice

#include "zit6_interfaces/msg/detail/zit_setpoint__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_zit6_interfaces
const rosidl_type_hash_t *
zit6_interfaces__msg__ZitSetpoint__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x79, 0x2f, 0x91, 0x6f, 0xb1, 0x54, 0xb3, 0x41,
      0x81, 0x98, 0x8d, 0x74, 0x1f, 0xb2, 0x1f, 0xe5,
      0x11, 0xd6, 0x4d, 0xbf, 0x2f, 0xf2, 0x22, 0x3d,
      0x35, 0x35, 0xaa, 0x16, 0xe2, 0xd3, 0xc2, 0x52,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char zit6_interfaces__msg__ZitSetpoint__TYPE_NAME[] = "zit6_interfaces/msg/ZitSetpoint";

// Define type names, field names, and default values
static char zit6_interfaces__msg__ZitSetpoint__FIELD_NAME__control_key[] = "control_key";
static char zit6_interfaces__msg__ZitSetpoint__FIELD_NAME__type_mask[] = "type_mask";
static char zit6_interfaces__msg__ZitSetpoint__FIELD_NAME__x[] = "x";
static char zit6_interfaces__msg__ZitSetpoint__FIELD_NAME__y[] = "y";
static char zit6_interfaces__msg__ZitSetpoint__FIELD_NAME__z[] = "z";
static char zit6_interfaces__msg__ZitSetpoint__FIELD_NAME__yaw[] = "yaw";
static char zit6_interfaces__msg__ZitSetpoint__FIELD_NAME__seq[] = "seq";

static rosidl_runtime_c__type_description__Field zit6_interfaces__msg__ZitSetpoint__FIELDS[] = {
  {
    {zit6_interfaces__msg__ZitSetpoint__FIELD_NAME__control_key, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitSetpoint__FIELD_NAME__type_mask, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitSetpoint__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitSetpoint__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitSetpoint__FIELD_NAME__z, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitSetpoint__FIELD_NAME__yaw, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitSetpoint__FIELD_NAME__seq, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
zit6_interfaces__msg__ZitSetpoint__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {zit6_interfaces__msg__ZitSetpoint__TYPE_NAME, 31, 31},
      {zit6_interfaces__msg__ZitSetpoint__FIELDS, 7, 7},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint8 control_key\n"
  "uint8 type_mask\n"
  "float32 x\n"
  "float32 y\n"
  "float32 z\n"
  "float32 yaw\n"
  "uint32 seq";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
zit6_interfaces__msg__ZitSetpoint__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {zit6_interfaces__msg__ZitSetpoint__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 87, 87},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
zit6_interfaces__msg__ZitSetpoint__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *zit6_interfaces__msg__ZitSetpoint__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

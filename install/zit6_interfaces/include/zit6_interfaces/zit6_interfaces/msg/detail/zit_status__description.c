// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from zit6_interfaces:msg/ZitStatus.idl
// generated code does not contain a copyright notice

#include "zit6_interfaces/msg/detail/zit_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_zit6_interfaces
const rosidl_type_hash_t *
zit6_interfaces__msg__ZitStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x98, 0x87, 0x82, 0xe6, 0x3d, 0x11, 0x9e, 0xde,
      0x1b, 0xf3, 0x18, 0x64, 0xab, 0x7b, 0x8d, 0x42,
      0xdd, 0x80, 0x8f, 0x2b, 0x05, 0x00, 0x50, 0x50,
      0x13, 0xe7, 0x60, 0x7f, 0xaf, 0xd7, 0xac, 0x65,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char zit6_interfaces__msg__ZitStatus__TYPE_NAME[] = "zit6_interfaces/msg/ZitStatus";

// Define type names, field names, and default values
static char zit6_interfaces__msg__ZitStatus__FIELD_NAME__is_armed[] = "is_armed";
static char zit6_interfaces__msg__ZitStatus__FIELD_NAME__control_level[] = "control_level";
static char zit6_interfaces__msg__ZitStatus__FIELD_NAME__navigation_ready[] = "navigation_ready";
static char zit6_interfaces__msg__ZitStatus__FIELD_NAME__forces[] = "forces";
static char zit6_interfaces__msg__ZitStatus__FIELD_NAME__cycle_time_ms[] = "cycle_time_ms";
static char zit6_interfaces__msg__ZitStatus__FIELD_NAME__battery_voltage[] = "battery_voltage";
static char zit6_interfaces__msg__ZitStatus__FIELD_NAME__error_flags[] = "error_flags";

static rosidl_runtime_c__type_description__Field zit6_interfaces__msg__ZitStatus__FIELDS[] = {
  {
    {zit6_interfaces__msg__ZitStatus__FIELD_NAME__is_armed, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitStatus__FIELD_NAME__control_level, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitStatus__FIELD_NAME__navigation_ready, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitStatus__FIELD_NAME__forces, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitStatus__FIELD_NAME__cycle_time_ms, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitStatus__FIELD_NAME__battery_voltage, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitStatus__FIELD_NAME__error_flags, 11, 11},
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
zit6_interfaces__msg__ZitStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {zit6_interfaces__msg__ZitStatus__TYPE_NAME, 29, 29},
      {zit6_interfaces__msg__ZitStatus__FIELDS, 7, 7},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "bool is_armed\n"
  "uint8 control_level\n"
  "bool navigation_ready\n"
  "float32[4] forces\n"
  "float32 cycle_time_ms\n"
  "float32 battery_voltage\n"
  "uint32 error_flags";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
zit6_interfaces__msg__ZitStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {zit6_interfaces__msg__ZitStatus__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 139, 139},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
zit6_interfaces__msg__ZitStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *zit6_interfaces__msg__ZitStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

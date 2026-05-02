// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from zit6_interfaces:msg/ZitSetpoint.idl
// generated code does not contain a copyright notice
#include "zit6_interfaces/msg/detail/zit_setpoint__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
zit6_interfaces__msg__ZitSetpoint__init(zit6_interfaces__msg__ZitSetpoint * msg)
{
  if (!msg) {
    return false;
  }
  // control_key
  // type_mask
  // x
  // y
  // z
  // yaw
  // seq
  return true;
}

void
zit6_interfaces__msg__ZitSetpoint__fini(zit6_interfaces__msg__ZitSetpoint * msg)
{
  if (!msg) {
    return;
  }
  // control_key
  // type_mask
  // x
  // y
  // z
  // yaw
  // seq
}

bool
zit6_interfaces__msg__ZitSetpoint__are_equal(const zit6_interfaces__msg__ZitSetpoint * lhs, const zit6_interfaces__msg__ZitSetpoint * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // control_key
  if (lhs->control_key != rhs->control_key) {
    return false;
  }
  // type_mask
  if (lhs->type_mask != rhs->type_mask) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // z
  if (lhs->z != rhs->z) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // seq
  if (lhs->seq != rhs->seq) {
    return false;
  }
  return true;
}

bool
zit6_interfaces__msg__ZitSetpoint__copy(
  const zit6_interfaces__msg__ZitSetpoint * input,
  zit6_interfaces__msg__ZitSetpoint * output)
{
  if (!input || !output) {
    return false;
  }
  // control_key
  output->control_key = input->control_key;
  // type_mask
  output->type_mask = input->type_mask;
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // z
  output->z = input->z;
  // yaw
  output->yaw = input->yaw;
  // seq
  output->seq = input->seq;
  return true;
}

zit6_interfaces__msg__ZitSetpoint *
zit6_interfaces__msg__ZitSetpoint__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zit6_interfaces__msg__ZitSetpoint * msg = (zit6_interfaces__msg__ZitSetpoint *)allocator.allocate(sizeof(zit6_interfaces__msg__ZitSetpoint), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(zit6_interfaces__msg__ZitSetpoint));
  bool success = zit6_interfaces__msg__ZitSetpoint__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
zit6_interfaces__msg__ZitSetpoint__destroy(zit6_interfaces__msg__ZitSetpoint * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    zit6_interfaces__msg__ZitSetpoint__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
zit6_interfaces__msg__ZitSetpoint__Sequence__init(zit6_interfaces__msg__ZitSetpoint__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zit6_interfaces__msg__ZitSetpoint * data = NULL;

  if (size) {
    data = (zit6_interfaces__msg__ZitSetpoint *)allocator.zero_allocate(size, sizeof(zit6_interfaces__msg__ZitSetpoint), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = zit6_interfaces__msg__ZitSetpoint__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        zit6_interfaces__msg__ZitSetpoint__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
zit6_interfaces__msg__ZitSetpoint__Sequence__fini(zit6_interfaces__msg__ZitSetpoint__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      zit6_interfaces__msg__ZitSetpoint__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

zit6_interfaces__msg__ZitSetpoint__Sequence *
zit6_interfaces__msg__ZitSetpoint__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zit6_interfaces__msg__ZitSetpoint__Sequence * array = (zit6_interfaces__msg__ZitSetpoint__Sequence *)allocator.allocate(sizeof(zit6_interfaces__msg__ZitSetpoint__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = zit6_interfaces__msg__ZitSetpoint__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
zit6_interfaces__msg__ZitSetpoint__Sequence__destroy(zit6_interfaces__msg__ZitSetpoint__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    zit6_interfaces__msg__ZitSetpoint__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
zit6_interfaces__msg__ZitSetpoint__Sequence__are_equal(const zit6_interfaces__msg__ZitSetpoint__Sequence * lhs, const zit6_interfaces__msg__ZitSetpoint__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!zit6_interfaces__msg__ZitSetpoint__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
zit6_interfaces__msg__ZitSetpoint__Sequence__copy(
  const zit6_interfaces__msg__ZitSetpoint__Sequence * input,
  zit6_interfaces__msg__ZitSetpoint__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(zit6_interfaces__msg__ZitSetpoint);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    zit6_interfaces__msg__ZitSetpoint * data =
      (zit6_interfaces__msg__ZitSetpoint *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!zit6_interfaces__msg__ZitSetpoint__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          zit6_interfaces__msg__ZitSetpoint__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!zit6_interfaces__msg__ZitSetpoint__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

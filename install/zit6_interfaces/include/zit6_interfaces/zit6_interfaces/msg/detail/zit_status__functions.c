// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from zit6_interfaces:msg/ZitStatus.idl
// generated code does not contain a copyright notice
#include "zit6_interfaces/msg/detail/zit_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
zit6_interfaces__msg__ZitStatus__init(zit6_interfaces__msg__ZitStatus * msg)
{
  if (!msg) {
    return false;
  }
  // is_armed
  // control_level
  // navigation_ready
  // forces
  // cycle_time_ms
  // battery_voltage
  // error_flags
  return true;
}

void
zit6_interfaces__msg__ZitStatus__fini(zit6_interfaces__msg__ZitStatus * msg)
{
  if (!msg) {
    return;
  }
  // is_armed
  // control_level
  // navigation_ready
  // forces
  // cycle_time_ms
  // battery_voltage
  // error_flags
}

bool
zit6_interfaces__msg__ZitStatus__are_equal(const zit6_interfaces__msg__ZitStatus * lhs, const zit6_interfaces__msg__ZitStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // is_armed
  if (lhs->is_armed != rhs->is_armed) {
    return false;
  }
  // control_level
  if (lhs->control_level != rhs->control_level) {
    return false;
  }
  // navigation_ready
  if (lhs->navigation_ready != rhs->navigation_ready) {
    return false;
  }
  // forces
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->forces[i] != rhs->forces[i]) {
      return false;
    }
  }
  // cycle_time_ms
  if (lhs->cycle_time_ms != rhs->cycle_time_ms) {
    return false;
  }
  // battery_voltage
  if (lhs->battery_voltage != rhs->battery_voltage) {
    return false;
  }
  // error_flags
  if (lhs->error_flags != rhs->error_flags) {
    return false;
  }
  return true;
}

bool
zit6_interfaces__msg__ZitStatus__copy(
  const zit6_interfaces__msg__ZitStatus * input,
  zit6_interfaces__msg__ZitStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // is_armed
  output->is_armed = input->is_armed;
  // control_level
  output->control_level = input->control_level;
  // navigation_ready
  output->navigation_ready = input->navigation_ready;
  // forces
  for (size_t i = 0; i < 4; ++i) {
    output->forces[i] = input->forces[i];
  }
  // cycle_time_ms
  output->cycle_time_ms = input->cycle_time_ms;
  // battery_voltage
  output->battery_voltage = input->battery_voltage;
  // error_flags
  output->error_flags = input->error_flags;
  return true;
}

zit6_interfaces__msg__ZitStatus *
zit6_interfaces__msg__ZitStatus__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zit6_interfaces__msg__ZitStatus * msg = (zit6_interfaces__msg__ZitStatus *)allocator.allocate(sizeof(zit6_interfaces__msg__ZitStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(zit6_interfaces__msg__ZitStatus));
  bool success = zit6_interfaces__msg__ZitStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
zit6_interfaces__msg__ZitStatus__destroy(zit6_interfaces__msg__ZitStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    zit6_interfaces__msg__ZitStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
zit6_interfaces__msg__ZitStatus__Sequence__init(zit6_interfaces__msg__ZitStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zit6_interfaces__msg__ZitStatus * data = NULL;

  if (size) {
    data = (zit6_interfaces__msg__ZitStatus *)allocator.zero_allocate(size, sizeof(zit6_interfaces__msg__ZitStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = zit6_interfaces__msg__ZitStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        zit6_interfaces__msg__ZitStatus__fini(&data[i - 1]);
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
zit6_interfaces__msg__ZitStatus__Sequence__fini(zit6_interfaces__msg__ZitStatus__Sequence * array)
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
      zit6_interfaces__msg__ZitStatus__fini(&array->data[i]);
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

zit6_interfaces__msg__ZitStatus__Sequence *
zit6_interfaces__msg__ZitStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zit6_interfaces__msg__ZitStatus__Sequence * array = (zit6_interfaces__msg__ZitStatus__Sequence *)allocator.allocate(sizeof(zit6_interfaces__msg__ZitStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = zit6_interfaces__msg__ZitStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
zit6_interfaces__msg__ZitStatus__Sequence__destroy(zit6_interfaces__msg__ZitStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    zit6_interfaces__msg__ZitStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
zit6_interfaces__msg__ZitStatus__Sequence__are_equal(const zit6_interfaces__msg__ZitStatus__Sequence * lhs, const zit6_interfaces__msg__ZitStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!zit6_interfaces__msg__ZitStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
zit6_interfaces__msg__ZitStatus__Sequence__copy(
  const zit6_interfaces__msg__ZitStatus__Sequence * input,
  zit6_interfaces__msg__ZitStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(zit6_interfaces__msg__ZitStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    zit6_interfaces__msg__ZitStatus * data =
      (zit6_interfaces__msg__ZitStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!zit6_interfaces__msg__ZitStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          zit6_interfaces__msg__ZitStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!zit6_interfaces__msg__ZitStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

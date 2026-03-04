// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from volumetric_drilling_msgs:msg/Index.idl
// generated code does not contain a copyright notice
#include "volumetric_drilling_msgs/msg/detail/index__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
volumetric_drilling_msgs__msg__Index__init(volumetric_drilling_msgs__msg__Index * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // z
  return true;
}

void
volumetric_drilling_msgs__msg__Index__fini(volumetric_drilling_msgs__msg__Index * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // z
}

bool
volumetric_drilling_msgs__msg__Index__are_equal(const volumetric_drilling_msgs__msg__Index * lhs, const volumetric_drilling_msgs__msg__Index * rhs)
{
  if (!lhs || !rhs) {
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
  return true;
}

bool
volumetric_drilling_msgs__msg__Index__copy(
  const volumetric_drilling_msgs__msg__Index * input,
  volumetric_drilling_msgs__msg__Index * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // z
  output->z = input->z;
  return true;
}

volumetric_drilling_msgs__msg__Index *
volumetric_drilling_msgs__msg__Index__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  volumetric_drilling_msgs__msg__Index * msg = (volumetric_drilling_msgs__msg__Index *)allocator.allocate(sizeof(volumetric_drilling_msgs__msg__Index), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(volumetric_drilling_msgs__msg__Index));
  bool success = volumetric_drilling_msgs__msg__Index__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
volumetric_drilling_msgs__msg__Index__destroy(volumetric_drilling_msgs__msg__Index * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    volumetric_drilling_msgs__msg__Index__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
volumetric_drilling_msgs__msg__Index__Sequence__init(volumetric_drilling_msgs__msg__Index__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  volumetric_drilling_msgs__msg__Index * data = NULL;

  if (size) {
    data = (volumetric_drilling_msgs__msg__Index *)allocator.zero_allocate(size, sizeof(volumetric_drilling_msgs__msg__Index), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = volumetric_drilling_msgs__msg__Index__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        volumetric_drilling_msgs__msg__Index__fini(&data[i - 1]);
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
volumetric_drilling_msgs__msg__Index__Sequence__fini(volumetric_drilling_msgs__msg__Index__Sequence * array)
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
      volumetric_drilling_msgs__msg__Index__fini(&array->data[i]);
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

volumetric_drilling_msgs__msg__Index__Sequence *
volumetric_drilling_msgs__msg__Index__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  volumetric_drilling_msgs__msg__Index__Sequence * array = (volumetric_drilling_msgs__msg__Index__Sequence *)allocator.allocate(sizeof(volumetric_drilling_msgs__msg__Index__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = volumetric_drilling_msgs__msg__Index__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
volumetric_drilling_msgs__msg__Index__Sequence__destroy(volumetric_drilling_msgs__msg__Index__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    volumetric_drilling_msgs__msg__Index__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
volumetric_drilling_msgs__msg__Index__Sequence__are_equal(const volumetric_drilling_msgs__msg__Index__Sequence * lhs, const volumetric_drilling_msgs__msg__Index__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!volumetric_drilling_msgs__msg__Index__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
volumetric_drilling_msgs__msg__Index__Sequence__copy(
  const volumetric_drilling_msgs__msg__Index__Sequence * input,
  volumetric_drilling_msgs__msg__Index__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(volumetric_drilling_msgs__msg__Index);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    volumetric_drilling_msgs__msg__Index * data =
      (volumetric_drilling_msgs__msg__Index *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!volumetric_drilling_msgs__msg__Index__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          volumetric_drilling_msgs__msg__Index__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!volumetric_drilling_msgs__msg__Index__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

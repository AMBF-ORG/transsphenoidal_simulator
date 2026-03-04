// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from volumetric_drilling_msgs:msg/Voxels.idl
// generated code does not contain a copyright notice
#include "volumetric_drilling_msgs/msg/detail/voxels__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `indices`
#include "volumetric_drilling_msgs/msg/detail/index__functions.h"
// Member `colors`
#include "std_msgs/msg/detail/color_rgba__functions.h"

bool
volumetric_drilling_msgs__msg__Voxels__init(volumetric_drilling_msgs__msg__Voxels * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    volumetric_drilling_msgs__msg__Voxels__fini(msg);
    return false;
  }
  // indices
  if (!volumetric_drilling_msgs__msg__Index__Sequence__init(&msg->indices, 0)) {
    volumetric_drilling_msgs__msg__Voxels__fini(msg);
    return false;
  }
  // colors
  if (!std_msgs__msg__ColorRGBA__Sequence__init(&msg->colors, 0)) {
    volumetric_drilling_msgs__msg__Voxels__fini(msg);
    return false;
  }
  return true;
}

void
volumetric_drilling_msgs__msg__Voxels__fini(volumetric_drilling_msgs__msg__Voxels * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // indices
  volumetric_drilling_msgs__msg__Index__Sequence__fini(&msg->indices);
  // colors
  std_msgs__msg__ColorRGBA__Sequence__fini(&msg->colors);
}

bool
volumetric_drilling_msgs__msg__Voxels__are_equal(const volumetric_drilling_msgs__msg__Voxels * lhs, const volumetric_drilling_msgs__msg__Voxels * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // indices
  if (!volumetric_drilling_msgs__msg__Index__Sequence__are_equal(
      &(lhs->indices), &(rhs->indices)))
  {
    return false;
  }
  // colors
  if (!std_msgs__msg__ColorRGBA__Sequence__are_equal(
      &(lhs->colors), &(rhs->colors)))
  {
    return false;
  }
  return true;
}

bool
volumetric_drilling_msgs__msg__Voxels__copy(
  const volumetric_drilling_msgs__msg__Voxels * input,
  volumetric_drilling_msgs__msg__Voxels * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // indices
  if (!volumetric_drilling_msgs__msg__Index__Sequence__copy(
      &(input->indices), &(output->indices)))
  {
    return false;
  }
  // colors
  if (!std_msgs__msg__ColorRGBA__Sequence__copy(
      &(input->colors), &(output->colors)))
  {
    return false;
  }
  return true;
}

volumetric_drilling_msgs__msg__Voxels *
volumetric_drilling_msgs__msg__Voxels__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  volumetric_drilling_msgs__msg__Voxels * msg = (volumetric_drilling_msgs__msg__Voxels *)allocator.allocate(sizeof(volumetric_drilling_msgs__msg__Voxels), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(volumetric_drilling_msgs__msg__Voxels));
  bool success = volumetric_drilling_msgs__msg__Voxels__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
volumetric_drilling_msgs__msg__Voxels__destroy(volumetric_drilling_msgs__msg__Voxels * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    volumetric_drilling_msgs__msg__Voxels__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
volumetric_drilling_msgs__msg__Voxels__Sequence__init(volumetric_drilling_msgs__msg__Voxels__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  volumetric_drilling_msgs__msg__Voxels * data = NULL;

  if (size) {
    data = (volumetric_drilling_msgs__msg__Voxels *)allocator.zero_allocate(size, sizeof(volumetric_drilling_msgs__msg__Voxels), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = volumetric_drilling_msgs__msg__Voxels__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        volumetric_drilling_msgs__msg__Voxels__fini(&data[i - 1]);
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
volumetric_drilling_msgs__msg__Voxels__Sequence__fini(volumetric_drilling_msgs__msg__Voxels__Sequence * array)
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
      volumetric_drilling_msgs__msg__Voxels__fini(&array->data[i]);
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

volumetric_drilling_msgs__msg__Voxels__Sequence *
volumetric_drilling_msgs__msg__Voxels__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  volumetric_drilling_msgs__msg__Voxels__Sequence * array = (volumetric_drilling_msgs__msg__Voxels__Sequence *)allocator.allocate(sizeof(volumetric_drilling_msgs__msg__Voxels__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = volumetric_drilling_msgs__msg__Voxels__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
volumetric_drilling_msgs__msg__Voxels__Sequence__destroy(volumetric_drilling_msgs__msg__Voxels__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    volumetric_drilling_msgs__msg__Voxels__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
volumetric_drilling_msgs__msg__Voxels__Sequence__are_equal(const volumetric_drilling_msgs__msg__Voxels__Sequence * lhs, const volumetric_drilling_msgs__msg__Voxels__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!volumetric_drilling_msgs__msg__Voxels__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
volumetric_drilling_msgs__msg__Voxels__Sequence__copy(
  const volumetric_drilling_msgs__msg__Voxels__Sequence * input,
  volumetric_drilling_msgs__msg__Voxels__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(volumetric_drilling_msgs__msg__Voxels);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    volumetric_drilling_msgs__msg__Voxels * data =
      (volumetric_drilling_msgs__msg__Voxels *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!volumetric_drilling_msgs__msg__Voxels__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          volumetric_drilling_msgs__msg__Voxels__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!volumetric_drilling_msgs__msg__Voxels__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

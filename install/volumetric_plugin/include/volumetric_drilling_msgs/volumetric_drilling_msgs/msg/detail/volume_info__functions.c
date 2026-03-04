// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from volumetric_drilling_msgs:msg/VolumeInfo.idl
// generated code does not contain a copyright notice
#include "volumetric_drilling_msgs/msg/detail/volume_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `dimensions`
// Member `voxel_count`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
volumetric_drilling_msgs__msg__VolumeInfo__init(volumetric_drilling_msgs__msg__VolumeInfo * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    volumetric_drilling_msgs__msg__VolumeInfo__fini(msg);
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    volumetric_drilling_msgs__msg__VolumeInfo__fini(msg);
    return false;
  }
  // dimensions
  if (!rosidl_runtime_c__float__Sequence__init(&msg->dimensions, 0)) {
    volumetric_drilling_msgs__msg__VolumeInfo__fini(msg);
    return false;
  }
  // voxel_count
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->voxel_count, 0)) {
    volumetric_drilling_msgs__msg__VolumeInfo__fini(msg);
    return false;
  }
  return true;
}

void
volumetric_drilling_msgs__msg__VolumeInfo__fini(volumetric_drilling_msgs__msg__VolumeInfo * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
  // dimensions
  rosidl_runtime_c__float__Sequence__fini(&msg->dimensions);
  // voxel_count
  rosidl_runtime_c__int32__Sequence__fini(&msg->voxel_count);
}

bool
volumetric_drilling_msgs__msg__VolumeInfo__are_equal(const volumetric_drilling_msgs__msg__VolumeInfo * lhs, const volumetric_drilling_msgs__msg__VolumeInfo * rhs)
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
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // dimensions
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->dimensions), &(rhs->dimensions)))
  {
    return false;
  }
  // voxel_count
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->voxel_count), &(rhs->voxel_count)))
  {
    return false;
  }
  return true;
}

bool
volumetric_drilling_msgs__msg__VolumeInfo__copy(
  const volumetric_drilling_msgs__msg__VolumeInfo * input,
  volumetric_drilling_msgs__msg__VolumeInfo * output)
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
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // dimensions
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->dimensions), &(output->dimensions)))
  {
    return false;
  }
  // voxel_count
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->voxel_count), &(output->voxel_count)))
  {
    return false;
  }
  return true;
}

volumetric_drilling_msgs__msg__VolumeInfo *
volumetric_drilling_msgs__msg__VolumeInfo__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  volumetric_drilling_msgs__msg__VolumeInfo * msg = (volumetric_drilling_msgs__msg__VolumeInfo *)allocator.allocate(sizeof(volumetric_drilling_msgs__msg__VolumeInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(volumetric_drilling_msgs__msg__VolumeInfo));
  bool success = volumetric_drilling_msgs__msg__VolumeInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
volumetric_drilling_msgs__msg__VolumeInfo__destroy(volumetric_drilling_msgs__msg__VolumeInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    volumetric_drilling_msgs__msg__VolumeInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
volumetric_drilling_msgs__msg__VolumeInfo__Sequence__init(volumetric_drilling_msgs__msg__VolumeInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  volumetric_drilling_msgs__msg__VolumeInfo * data = NULL;

  if (size) {
    data = (volumetric_drilling_msgs__msg__VolumeInfo *)allocator.zero_allocate(size, sizeof(volumetric_drilling_msgs__msg__VolumeInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = volumetric_drilling_msgs__msg__VolumeInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        volumetric_drilling_msgs__msg__VolumeInfo__fini(&data[i - 1]);
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
volumetric_drilling_msgs__msg__VolumeInfo__Sequence__fini(volumetric_drilling_msgs__msg__VolumeInfo__Sequence * array)
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
      volumetric_drilling_msgs__msg__VolumeInfo__fini(&array->data[i]);
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

volumetric_drilling_msgs__msg__VolumeInfo__Sequence *
volumetric_drilling_msgs__msg__VolumeInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  volumetric_drilling_msgs__msg__VolumeInfo__Sequence * array = (volumetric_drilling_msgs__msg__VolumeInfo__Sequence *)allocator.allocate(sizeof(volumetric_drilling_msgs__msg__VolumeInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = volumetric_drilling_msgs__msg__VolumeInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
volumetric_drilling_msgs__msg__VolumeInfo__Sequence__destroy(volumetric_drilling_msgs__msg__VolumeInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    volumetric_drilling_msgs__msg__VolumeInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
volumetric_drilling_msgs__msg__VolumeInfo__Sequence__are_equal(const volumetric_drilling_msgs__msg__VolumeInfo__Sequence * lhs, const volumetric_drilling_msgs__msg__VolumeInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!volumetric_drilling_msgs__msg__VolumeInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
volumetric_drilling_msgs__msg__VolumeInfo__Sequence__copy(
  const volumetric_drilling_msgs__msg__VolumeInfo__Sequence * input,
  volumetric_drilling_msgs__msg__VolumeInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(volumetric_drilling_msgs__msg__VolumeInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    volumetric_drilling_msgs__msg__VolumeInfo * data =
      (volumetric_drilling_msgs__msg__VolumeInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!volumetric_drilling_msgs__msg__VolumeInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          volumetric_drilling_msgs__msg__VolumeInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!volumetric_drilling_msgs__msg__VolumeInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from volumetric_drilling_msgs:msg/Voxels.idl
// generated code does not contain a copyright notice

#ifndef VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__VOXELS__STRUCT_H_
#define VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__VOXELS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'indices'
#include "volumetric_drilling_msgs/msg/detail/index__struct.h"
// Member 'colors'
#include "std_msgs/msg/detail/color_rgba__struct.h"

/// Struct defined in msg/Voxels in the package volumetric_drilling_msgs.
typedef struct volumetric_drilling_msgs__msg__Voxels
{
  std_msgs__msg__Header header;
  volumetric_drilling_msgs__msg__Index__Sequence indices;
  std_msgs__msg__ColorRGBA__Sequence colors;
} volumetric_drilling_msgs__msg__Voxels;

// Struct for a sequence of volumetric_drilling_msgs__msg__Voxels.
typedef struct volumetric_drilling_msgs__msg__Voxels__Sequence
{
  volumetric_drilling_msgs__msg__Voxels * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} volumetric_drilling_msgs__msg__Voxels__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__VOXELS__STRUCT_H_

// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from volumetric_drilling_msgs:msg/VolumeInfo.idl
// generated code does not contain a copyright notice

#ifndef VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__VOLUME_INFO__STRUCT_H_
#define VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__VOLUME_INFO__STRUCT_H_

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
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'dimensions'
// Member 'voxel_count'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/VolumeInfo in the package volumetric_drilling_msgs.
typedef struct volumetric_drilling_msgs__msg__VolumeInfo
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__Pose pose;
  rosidl_runtime_c__float__Sequence dimensions;
  rosidl_runtime_c__int32__Sequence voxel_count;
} volumetric_drilling_msgs__msg__VolumeInfo;

// Struct for a sequence of volumetric_drilling_msgs__msg__VolumeInfo.
typedef struct volumetric_drilling_msgs__msg__VolumeInfo__Sequence
{
  volumetric_drilling_msgs__msg__VolumeInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} volumetric_drilling_msgs__msg__VolumeInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__VOLUME_INFO__STRUCT_H_

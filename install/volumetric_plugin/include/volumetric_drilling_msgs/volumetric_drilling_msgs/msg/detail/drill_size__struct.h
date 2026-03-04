// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from volumetric_drilling_msgs:msg/DrillSize.idl
// generated code does not contain a copyright notice

#ifndef VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__DRILL_SIZE__STRUCT_H_
#define VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__DRILL_SIZE__STRUCT_H_

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
// Member 'size'
#include "std_msgs/msg/detail/u_int8__struct.h"

/// Struct defined in msg/DrillSize in the package volumetric_drilling_msgs.
typedef struct volumetric_drilling_msgs__msg__DrillSize
{
  std_msgs__msg__Header header;
  std_msgs__msg__UInt8 size;
} volumetric_drilling_msgs__msg__DrillSize;

// Struct for a sequence of volumetric_drilling_msgs__msg__DrillSize.
typedef struct volumetric_drilling_msgs__msg__DrillSize__Sequence
{
  volumetric_drilling_msgs__msg__DrillSize * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} volumetric_drilling_msgs__msg__DrillSize__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__DRILL_SIZE__STRUCT_H_

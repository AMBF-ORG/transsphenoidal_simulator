// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from volumetric_drilling_msgs:msg/Index.idl
// generated code does not contain a copyright notice

#ifndef VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__INDEX__STRUCT_H_
#define VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__INDEX__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Index in the package volumetric_drilling_msgs.
typedef struct volumetric_drilling_msgs__msg__Index
{
  int64_t x;
  int64_t y;
  int64_t z;
} volumetric_drilling_msgs__msg__Index;

// Struct for a sequence of volumetric_drilling_msgs__msg__Index.
typedef struct volumetric_drilling_msgs__msg__Index__Sequence
{
  volumetric_drilling_msgs__msg__Index * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} volumetric_drilling_msgs__msg__Index__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__INDEX__STRUCT_H_

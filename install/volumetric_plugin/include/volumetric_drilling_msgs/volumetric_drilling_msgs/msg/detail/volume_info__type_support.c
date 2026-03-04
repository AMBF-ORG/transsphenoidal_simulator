// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from volumetric_drilling_msgs:msg/VolumeInfo.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "volumetric_drilling_msgs/msg/detail/volume_info__rosidl_typesupport_introspection_c.h"
#include "volumetric_drilling_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "volumetric_drilling_msgs/msg/detail/volume_info__functions.h"
#include "volumetric_drilling_msgs/msg/detail/volume_info__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `pose`
#include "geometry_msgs/msg/pose.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"
// Member `dimensions`
// Member `voxel_count`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__VolumeInfo_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  volumetric_drilling_msgs__msg__VolumeInfo__init(message_memory);
}

void volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__VolumeInfo_fini_function(void * message_memory)
{
  volumetric_drilling_msgs__msg__VolumeInfo__fini(message_memory);
}

size_t volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__size_function__VolumeInfo__dimensions(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__get_const_function__VolumeInfo__dimensions(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__get_function__VolumeInfo__dimensions(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__fetch_function__VolumeInfo__dimensions(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__get_const_function__VolumeInfo__dimensions(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__assign_function__VolumeInfo__dimensions(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__get_function__VolumeInfo__dimensions(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__resize_function__VolumeInfo__dimensions(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__size_function__VolumeInfo__voxel_count(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__get_const_function__VolumeInfo__voxel_count(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__get_function__VolumeInfo__voxel_count(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__fetch_function__VolumeInfo__voxel_count(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__get_const_function__VolumeInfo__voxel_count(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__assign_function__VolumeInfo__voxel_count(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__get_function__VolumeInfo__voxel_count(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__resize_function__VolumeInfo__voxel_count(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__VolumeInfo_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(volumetric_drilling_msgs__msg__VolumeInfo, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(volumetric_drilling_msgs__msg__VolumeInfo, pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "dimensions",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(volumetric_drilling_msgs__msg__VolumeInfo, dimensions),  // bytes offset in struct
    NULL,  // default value
    volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__size_function__VolumeInfo__dimensions,  // size() function pointer
    volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__get_const_function__VolumeInfo__dimensions,  // get_const(index) function pointer
    volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__get_function__VolumeInfo__dimensions,  // get(index) function pointer
    volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__fetch_function__VolumeInfo__dimensions,  // fetch(index, &value) function pointer
    volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__assign_function__VolumeInfo__dimensions,  // assign(index, value) function pointer
    volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__resize_function__VolumeInfo__dimensions  // resize(index) function pointer
  },
  {
    "voxel_count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(volumetric_drilling_msgs__msg__VolumeInfo, voxel_count),  // bytes offset in struct
    NULL,  // default value
    volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__size_function__VolumeInfo__voxel_count,  // size() function pointer
    volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__get_const_function__VolumeInfo__voxel_count,  // get_const(index) function pointer
    volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__get_function__VolumeInfo__voxel_count,  // get(index) function pointer
    volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__fetch_function__VolumeInfo__voxel_count,  // fetch(index, &value) function pointer
    volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__assign_function__VolumeInfo__voxel_count,  // assign(index, value) function pointer
    volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__resize_function__VolumeInfo__voxel_count  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__VolumeInfo_message_members = {
  "volumetric_drilling_msgs__msg",  // message namespace
  "VolumeInfo",  // message name
  4,  // number of fields
  sizeof(volumetric_drilling_msgs__msg__VolumeInfo),
  volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__VolumeInfo_message_member_array,  // message members
  volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__VolumeInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__VolumeInfo_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__VolumeInfo_message_type_support_handle = {
  0,
  &volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__VolumeInfo_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_volumetric_drilling_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, volumetric_drilling_msgs, msg, VolumeInfo)() {
  volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__VolumeInfo_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__VolumeInfo_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  if (!volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__VolumeInfo_message_type_support_handle.typesupport_identifier) {
    volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__VolumeInfo_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &volumetric_drilling_msgs__msg__VolumeInfo__rosidl_typesupport_introspection_c__VolumeInfo_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

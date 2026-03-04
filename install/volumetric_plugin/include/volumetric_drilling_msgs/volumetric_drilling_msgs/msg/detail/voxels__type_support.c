// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from volumetric_drilling_msgs:msg/Voxels.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "volumetric_drilling_msgs/msg/detail/voxels__rosidl_typesupport_introspection_c.h"
#include "volumetric_drilling_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "volumetric_drilling_msgs/msg/detail/voxels__functions.h"
#include "volumetric_drilling_msgs/msg/detail/voxels__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `indices`
#include "volumetric_drilling_msgs/msg/index.h"
// Member `indices`
#include "volumetric_drilling_msgs/msg/detail/index__rosidl_typesupport_introspection_c.h"
// Member `colors`
#include "std_msgs/msg/color_rgba.h"
// Member `colors`
#include "std_msgs/msg/detail/color_rgba__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__Voxels_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  volumetric_drilling_msgs__msg__Voxels__init(message_memory);
}

void volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__Voxels_fini_function(void * message_memory)
{
  volumetric_drilling_msgs__msg__Voxels__fini(message_memory);
}

size_t volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__size_function__Voxels__indices(
  const void * untyped_member)
{
  const volumetric_drilling_msgs__msg__Index__Sequence * member =
    (const volumetric_drilling_msgs__msg__Index__Sequence *)(untyped_member);
  return member->size;
}

const void * volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__get_const_function__Voxels__indices(
  const void * untyped_member, size_t index)
{
  const volumetric_drilling_msgs__msg__Index__Sequence * member =
    (const volumetric_drilling_msgs__msg__Index__Sequence *)(untyped_member);
  return &member->data[index];
}

void * volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__get_function__Voxels__indices(
  void * untyped_member, size_t index)
{
  volumetric_drilling_msgs__msg__Index__Sequence * member =
    (volumetric_drilling_msgs__msg__Index__Sequence *)(untyped_member);
  return &member->data[index];
}

void volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__fetch_function__Voxels__indices(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const volumetric_drilling_msgs__msg__Index * item =
    ((const volumetric_drilling_msgs__msg__Index *)
    volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__get_const_function__Voxels__indices(untyped_member, index));
  volumetric_drilling_msgs__msg__Index * value =
    (volumetric_drilling_msgs__msg__Index *)(untyped_value);
  *value = *item;
}

void volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__assign_function__Voxels__indices(
  void * untyped_member, size_t index, const void * untyped_value)
{
  volumetric_drilling_msgs__msg__Index * item =
    ((volumetric_drilling_msgs__msg__Index *)
    volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__get_function__Voxels__indices(untyped_member, index));
  const volumetric_drilling_msgs__msg__Index * value =
    (const volumetric_drilling_msgs__msg__Index *)(untyped_value);
  *item = *value;
}

bool volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__resize_function__Voxels__indices(
  void * untyped_member, size_t size)
{
  volumetric_drilling_msgs__msg__Index__Sequence * member =
    (volumetric_drilling_msgs__msg__Index__Sequence *)(untyped_member);
  volumetric_drilling_msgs__msg__Index__Sequence__fini(member);
  return volumetric_drilling_msgs__msg__Index__Sequence__init(member, size);
}

size_t volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__size_function__Voxels__colors(
  const void * untyped_member)
{
  const std_msgs__msg__ColorRGBA__Sequence * member =
    (const std_msgs__msg__ColorRGBA__Sequence *)(untyped_member);
  return member->size;
}

const void * volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__get_const_function__Voxels__colors(
  const void * untyped_member, size_t index)
{
  const std_msgs__msg__ColorRGBA__Sequence * member =
    (const std_msgs__msg__ColorRGBA__Sequence *)(untyped_member);
  return &member->data[index];
}

void * volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__get_function__Voxels__colors(
  void * untyped_member, size_t index)
{
  std_msgs__msg__ColorRGBA__Sequence * member =
    (std_msgs__msg__ColorRGBA__Sequence *)(untyped_member);
  return &member->data[index];
}

void volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__fetch_function__Voxels__colors(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const std_msgs__msg__ColorRGBA * item =
    ((const std_msgs__msg__ColorRGBA *)
    volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__get_const_function__Voxels__colors(untyped_member, index));
  std_msgs__msg__ColorRGBA * value =
    (std_msgs__msg__ColorRGBA *)(untyped_value);
  *value = *item;
}

void volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__assign_function__Voxels__colors(
  void * untyped_member, size_t index, const void * untyped_value)
{
  std_msgs__msg__ColorRGBA * item =
    ((std_msgs__msg__ColorRGBA *)
    volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__get_function__Voxels__colors(untyped_member, index));
  const std_msgs__msg__ColorRGBA * value =
    (const std_msgs__msg__ColorRGBA *)(untyped_value);
  *item = *value;
}

bool volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__resize_function__Voxels__colors(
  void * untyped_member, size_t size)
{
  std_msgs__msg__ColorRGBA__Sequence * member =
    (std_msgs__msg__ColorRGBA__Sequence *)(untyped_member);
  std_msgs__msg__ColorRGBA__Sequence__fini(member);
  return std_msgs__msg__ColorRGBA__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__Voxels_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(volumetric_drilling_msgs__msg__Voxels, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "indices",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(volumetric_drilling_msgs__msg__Voxels, indices),  // bytes offset in struct
    NULL,  // default value
    volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__size_function__Voxels__indices,  // size() function pointer
    volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__get_const_function__Voxels__indices,  // get_const(index) function pointer
    volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__get_function__Voxels__indices,  // get(index) function pointer
    volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__fetch_function__Voxels__indices,  // fetch(index, &value) function pointer
    volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__assign_function__Voxels__indices,  // assign(index, value) function pointer
    volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__resize_function__Voxels__indices  // resize(index) function pointer
  },
  {
    "colors",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(volumetric_drilling_msgs__msg__Voxels, colors),  // bytes offset in struct
    NULL,  // default value
    volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__size_function__Voxels__colors,  // size() function pointer
    volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__get_const_function__Voxels__colors,  // get_const(index) function pointer
    volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__get_function__Voxels__colors,  // get(index) function pointer
    volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__fetch_function__Voxels__colors,  // fetch(index, &value) function pointer
    volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__assign_function__Voxels__colors,  // assign(index, value) function pointer
    volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__resize_function__Voxels__colors  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__Voxels_message_members = {
  "volumetric_drilling_msgs__msg",  // message namespace
  "Voxels",  // message name
  3,  // number of fields
  sizeof(volumetric_drilling_msgs__msg__Voxels),
  volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__Voxels_message_member_array,  // message members
  volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__Voxels_init_function,  // function to initialize message memory (memory has to be allocated)
  volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__Voxels_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__Voxels_message_type_support_handle = {
  0,
  &volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__Voxels_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_volumetric_drilling_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, volumetric_drilling_msgs, msg, Voxels)() {
  volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__Voxels_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__Voxels_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, volumetric_drilling_msgs, msg, Index)();
  volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__Voxels_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, ColorRGBA)();
  if (!volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__Voxels_message_type_support_handle.typesupport_identifier) {
    volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__Voxels_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &volumetric_drilling_msgs__msg__Voxels__rosidl_typesupport_introspection_c__Voxels_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

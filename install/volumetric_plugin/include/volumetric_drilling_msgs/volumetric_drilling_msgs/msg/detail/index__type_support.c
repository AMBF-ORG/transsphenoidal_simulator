// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from volumetric_drilling_msgs:msg/Index.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "volumetric_drilling_msgs/msg/detail/index__rosidl_typesupport_introspection_c.h"
#include "volumetric_drilling_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "volumetric_drilling_msgs/msg/detail/index__functions.h"
#include "volumetric_drilling_msgs/msg/detail/index__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void volumetric_drilling_msgs__msg__Index__rosidl_typesupport_introspection_c__Index_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  volumetric_drilling_msgs__msg__Index__init(message_memory);
}

void volumetric_drilling_msgs__msg__Index__rosidl_typesupport_introspection_c__Index_fini_function(void * message_memory)
{
  volumetric_drilling_msgs__msg__Index__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember volumetric_drilling_msgs__msg__Index__rosidl_typesupport_introspection_c__Index_message_member_array[3] = {
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(volumetric_drilling_msgs__msg__Index, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(volumetric_drilling_msgs__msg__Index, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(volumetric_drilling_msgs__msg__Index, z),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers volumetric_drilling_msgs__msg__Index__rosidl_typesupport_introspection_c__Index_message_members = {
  "volumetric_drilling_msgs__msg",  // message namespace
  "Index",  // message name
  3,  // number of fields
  sizeof(volumetric_drilling_msgs__msg__Index),
  volumetric_drilling_msgs__msg__Index__rosidl_typesupport_introspection_c__Index_message_member_array,  // message members
  volumetric_drilling_msgs__msg__Index__rosidl_typesupport_introspection_c__Index_init_function,  // function to initialize message memory (memory has to be allocated)
  volumetric_drilling_msgs__msg__Index__rosidl_typesupport_introspection_c__Index_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t volumetric_drilling_msgs__msg__Index__rosidl_typesupport_introspection_c__Index_message_type_support_handle = {
  0,
  &volumetric_drilling_msgs__msg__Index__rosidl_typesupport_introspection_c__Index_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_volumetric_drilling_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, volumetric_drilling_msgs, msg, Index)() {
  if (!volumetric_drilling_msgs__msg__Index__rosidl_typesupport_introspection_c__Index_message_type_support_handle.typesupport_identifier) {
    volumetric_drilling_msgs__msg__Index__rosidl_typesupport_introspection_c__Index_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &volumetric_drilling_msgs__msg__Index__rosidl_typesupport_introspection_c__Index_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

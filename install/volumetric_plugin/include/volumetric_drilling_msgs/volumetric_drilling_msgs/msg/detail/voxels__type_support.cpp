// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from volumetric_drilling_msgs:msg/Voxels.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "volumetric_drilling_msgs/msg/detail/voxels__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace volumetric_drilling_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Voxels_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) volumetric_drilling_msgs::msg::Voxels(_init);
}

void Voxels_fini_function(void * message_memory)
{
  auto typed_message = static_cast<volumetric_drilling_msgs::msg::Voxels *>(message_memory);
  typed_message->~Voxels();
}

size_t size_function__Voxels__indices(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<volumetric_drilling_msgs::msg::Index> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Voxels__indices(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<volumetric_drilling_msgs::msg::Index> *>(untyped_member);
  return &member[index];
}

void * get_function__Voxels__indices(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<volumetric_drilling_msgs::msg::Index> *>(untyped_member);
  return &member[index];
}

void fetch_function__Voxels__indices(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const volumetric_drilling_msgs::msg::Index *>(
    get_const_function__Voxels__indices(untyped_member, index));
  auto & value = *reinterpret_cast<volumetric_drilling_msgs::msg::Index *>(untyped_value);
  value = item;
}

void assign_function__Voxels__indices(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<volumetric_drilling_msgs::msg::Index *>(
    get_function__Voxels__indices(untyped_member, index));
  const auto & value = *reinterpret_cast<const volumetric_drilling_msgs::msg::Index *>(untyped_value);
  item = value;
}

void resize_function__Voxels__indices(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<volumetric_drilling_msgs::msg::Index> *>(untyped_member);
  member->resize(size);
}

size_t size_function__Voxels__colors(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std_msgs::msg::ColorRGBA> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Voxels__colors(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std_msgs::msg::ColorRGBA> *>(untyped_member);
  return &member[index];
}

void * get_function__Voxels__colors(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std_msgs::msg::ColorRGBA> *>(untyped_member);
  return &member[index];
}

void fetch_function__Voxels__colors(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std_msgs::msg::ColorRGBA *>(
    get_const_function__Voxels__colors(untyped_member, index));
  auto & value = *reinterpret_cast<std_msgs::msg::ColorRGBA *>(untyped_value);
  value = item;
}

void assign_function__Voxels__colors(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std_msgs::msg::ColorRGBA *>(
    get_function__Voxels__colors(untyped_member, index));
  const auto & value = *reinterpret_cast<const std_msgs::msg::ColorRGBA *>(untyped_value);
  item = value;
}

void resize_function__Voxels__colors(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std_msgs::msg::ColorRGBA> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Voxels_message_member_array[3] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(volumetric_drilling_msgs::msg::Voxels, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "indices",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<volumetric_drilling_msgs::msg::Index>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(volumetric_drilling_msgs::msg::Voxels, indices),  // bytes offset in struct
    nullptr,  // default value
    size_function__Voxels__indices,  // size() function pointer
    get_const_function__Voxels__indices,  // get_const(index) function pointer
    get_function__Voxels__indices,  // get(index) function pointer
    fetch_function__Voxels__indices,  // fetch(index, &value) function pointer
    assign_function__Voxels__indices,  // assign(index, value) function pointer
    resize_function__Voxels__indices  // resize(index) function pointer
  },
  {
    "colors",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::ColorRGBA>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(volumetric_drilling_msgs::msg::Voxels, colors),  // bytes offset in struct
    nullptr,  // default value
    size_function__Voxels__colors,  // size() function pointer
    get_const_function__Voxels__colors,  // get_const(index) function pointer
    get_function__Voxels__colors,  // get(index) function pointer
    fetch_function__Voxels__colors,  // fetch(index, &value) function pointer
    assign_function__Voxels__colors,  // assign(index, value) function pointer
    resize_function__Voxels__colors  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Voxels_message_members = {
  "volumetric_drilling_msgs::msg",  // message namespace
  "Voxels",  // message name
  3,  // number of fields
  sizeof(volumetric_drilling_msgs::msg::Voxels),
  Voxels_message_member_array,  // message members
  Voxels_init_function,  // function to initialize message memory (memory has to be allocated)
  Voxels_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Voxels_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Voxels_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace volumetric_drilling_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<volumetric_drilling_msgs::msg::Voxels>()
{
  return &::volumetric_drilling_msgs::msg::rosidl_typesupport_introspection_cpp::Voxels_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, volumetric_drilling_msgs, msg, Voxels)() {
  return &::volumetric_drilling_msgs::msg::rosidl_typesupport_introspection_cpp::Voxels_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

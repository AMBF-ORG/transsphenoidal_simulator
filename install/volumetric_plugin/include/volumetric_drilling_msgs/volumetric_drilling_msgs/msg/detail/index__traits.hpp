// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from volumetric_drilling_msgs:msg/Index.idl
// generated code does not contain a copyright notice

#ifndef VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__INDEX__TRAITS_HPP_
#define VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__INDEX__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "volumetric_drilling_msgs/msg/detail/index__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace volumetric_drilling_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Index & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: z
  {
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Index & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Index & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace volumetric_drilling_msgs

namespace rosidl_generator_traits
{

[[deprecated("use volumetric_drilling_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const volumetric_drilling_msgs::msg::Index & msg,
  std::ostream & out, size_t indentation = 0)
{
  volumetric_drilling_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use volumetric_drilling_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const volumetric_drilling_msgs::msg::Index & msg)
{
  return volumetric_drilling_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<volumetric_drilling_msgs::msg::Index>()
{
  return "volumetric_drilling_msgs::msg::Index";
}

template<>
inline const char * name<volumetric_drilling_msgs::msg::Index>()
{
  return "volumetric_drilling_msgs/msg/Index";
}

template<>
struct has_fixed_size<volumetric_drilling_msgs::msg::Index>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<volumetric_drilling_msgs::msg::Index>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<volumetric_drilling_msgs::msg::Index>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__INDEX__TRAITS_HPP_

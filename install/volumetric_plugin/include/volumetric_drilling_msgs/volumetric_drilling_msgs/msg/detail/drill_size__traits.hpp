// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from volumetric_drilling_msgs:msg/DrillSize.idl
// generated code does not contain a copyright notice

#ifndef VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__DRILL_SIZE__TRAITS_HPP_
#define VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__DRILL_SIZE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "volumetric_drilling_msgs/msg/detail/drill_size__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'size'
#include "std_msgs/msg/detail/u_int8__traits.hpp"

namespace volumetric_drilling_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const DrillSize & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: size
  {
    out << "size: ";
    to_flow_style_yaml(msg.size, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DrillSize & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: size
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "size:\n";
    to_block_style_yaml(msg.size, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DrillSize & msg, bool use_flow_style = false)
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
  const volumetric_drilling_msgs::msg::DrillSize & msg,
  std::ostream & out, size_t indentation = 0)
{
  volumetric_drilling_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use volumetric_drilling_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const volumetric_drilling_msgs::msg::DrillSize & msg)
{
  return volumetric_drilling_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<volumetric_drilling_msgs::msg::DrillSize>()
{
  return "volumetric_drilling_msgs::msg::DrillSize";
}

template<>
inline const char * name<volumetric_drilling_msgs::msg::DrillSize>()
{
  return "volumetric_drilling_msgs/msg/DrillSize";
}

template<>
struct has_fixed_size<volumetric_drilling_msgs::msg::DrillSize>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value && has_fixed_size<std_msgs::msg::UInt8>::value> {};

template<>
struct has_bounded_size<volumetric_drilling_msgs::msg::DrillSize>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value && has_bounded_size<std_msgs::msg::UInt8>::value> {};

template<>
struct is_message<volumetric_drilling_msgs::msg::DrillSize>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__DRILL_SIZE__TRAITS_HPP_

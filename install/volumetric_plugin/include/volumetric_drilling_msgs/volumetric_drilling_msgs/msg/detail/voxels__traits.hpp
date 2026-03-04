// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from volumetric_drilling_msgs:msg/Voxels.idl
// generated code does not contain a copyright notice

#ifndef VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__VOXELS__TRAITS_HPP_
#define VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__VOXELS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "volumetric_drilling_msgs/msg/detail/voxels__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'indices'
#include "volumetric_drilling_msgs/msg/detail/index__traits.hpp"
// Member 'colors'
#include "std_msgs/msg/detail/color_rgba__traits.hpp"

namespace volumetric_drilling_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Voxels & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: indices
  {
    if (msg.indices.size() == 0) {
      out << "indices: []";
    } else {
      out << "indices: [";
      size_t pending_items = msg.indices.size();
      for (auto item : msg.indices) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: colors
  {
    if (msg.colors.size() == 0) {
      out << "colors: []";
    } else {
      out << "colors: [";
      size_t pending_items = msg.colors.size();
      for (auto item : msg.colors) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Voxels & msg,
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

  // member: indices
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.indices.size() == 0) {
      out << "indices: []\n";
    } else {
      out << "indices:\n";
      for (auto item : msg.indices) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: colors
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.colors.size() == 0) {
      out << "colors: []\n";
    } else {
      out << "colors:\n";
      for (auto item : msg.colors) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Voxels & msg, bool use_flow_style = false)
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
  const volumetric_drilling_msgs::msg::Voxels & msg,
  std::ostream & out, size_t indentation = 0)
{
  volumetric_drilling_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use volumetric_drilling_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const volumetric_drilling_msgs::msg::Voxels & msg)
{
  return volumetric_drilling_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<volumetric_drilling_msgs::msg::Voxels>()
{
  return "volumetric_drilling_msgs::msg::Voxels";
}

template<>
inline const char * name<volumetric_drilling_msgs::msg::Voxels>()
{
  return "volumetric_drilling_msgs/msg/Voxels";
}

template<>
struct has_fixed_size<volumetric_drilling_msgs::msg::Voxels>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<volumetric_drilling_msgs::msg::Voxels>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<volumetric_drilling_msgs::msg::Voxels>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__VOXELS__TRAITS_HPP_

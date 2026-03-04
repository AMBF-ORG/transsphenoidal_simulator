// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from volumetric_drilling_msgs:msg/VolumeInfo.idl
// generated code does not contain a copyright notice

#ifndef VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__VOLUME_INFO__TRAITS_HPP_
#define VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__VOLUME_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "volumetric_drilling_msgs/msg/detail/volume_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace volumetric_drilling_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const VolumeInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
    out << ", ";
  }

  // member: dimensions
  {
    if (msg.dimensions.size() == 0) {
      out << "dimensions: []";
    } else {
      out << "dimensions: [";
      size_t pending_items = msg.dimensions.size();
      for (auto item : msg.dimensions) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: voxel_count
  {
    if (msg.voxel_count.size() == 0) {
      out << "voxel_count: []";
    } else {
      out << "voxel_count: [";
      size_t pending_items = msg.voxel_count.size();
      for (auto item : msg.voxel_count) {
        rosidl_generator_traits::value_to_yaml(item, out);
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
  const VolumeInfo & msg,
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

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }

  // member: dimensions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.dimensions.size() == 0) {
      out << "dimensions: []\n";
    } else {
      out << "dimensions:\n";
      for (auto item : msg.dimensions) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: voxel_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.voxel_count.size() == 0) {
      out << "voxel_count: []\n";
    } else {
      out << "voxel_count:\n";
      for (auto item : msg.voxel_count) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const VolumeInfo & msg, bool use_flow_style = false)
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
  const volumetric_drilling_msgs::msg::VolumeInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  volumetric_drilling_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use volumetric_drilling_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const volumetric_drilling_msgs::msg::VolumeInfo & msg)
{
  return volumetric_drilling_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<volumetric_drilling_msgs::msg::VolumeInfo>()
{
  return "volumetric_drilling_msgs::msg::VolumeInfo";
}

template<>
inline const char * name<volumetric_drilling_msgs::msg::VolumeInfo>()
{
  return "volumetric_drilling_msgs/msg/VolumeInfo";
}

template<>
struct has_fixed_size<volumetric_drilling_msgs::msg::VolumeInfo>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<volumetric_drilling_msgs::msg::VolumeInfo>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<volumetric_drilling_msgs::msg::VolumeInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__VOLUME_INFO__TRAITS_HPP_

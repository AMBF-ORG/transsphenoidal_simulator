// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from volumetric_drilling_msgs:msg/Voxels.idl
// generated code does not contain a copyright notice

#ifndef VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__VOXELS__BUILDER_HPP_
#define VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__VOXELS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "volumetric_drilling_msgs/msg/detail/voxels__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace volumetric_drilling_msgs
{

namespace msg
{

namespace builder
{

class Init_Voxels_colors
{
public:
  explicit Init_Voxels_colors(::volumetric_drilling_msgs::msg::Voxels & msg)
  : msg_(msg)
  {}
  ::volumetric_drilling_msgs::msg::Voxels colors(::volumetric_drilling_msgs::msg::Voxels::_colors_type arg)
  {
    msg_.colors = std::move(arg);
    return std::move(msg_);
  }

private:
  ::volumetric_drilling_msgs::msg::Voxels msg_;
};

class Init_Voxels_indices
{
public:
  explicit Init_Voxels_indices(::volumetric_drilling_msgs::msg::Voxels & msg)
  : msg_(msg)
  {}
  Init_Voxels_colors indices(::volumetric_drilling_msgs::msg::Voxels::_indices_type arg)
  {
    msg_.indices = std::move(arg);
    return Init_Voxels_colors(msg_);
  }

private:
  ::volumetric_drilling_msgs::msg::Voxels msg_;
};

class Init_Voxels_header
{
public:
  Init_Voxels_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Voxels_indices header(::volumetric_drilling_msgs::msg::Voxels::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Voxels_indices(msg_);
  }

private:
  ::volumetric_drilling_msgs::msg::Voxels msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::volumetric_drilling_msgs::msg::Voxels>()
{
  return volumetric_drilling_msgs::msg::builder::Init_Voxels_header();
}

}  // namespace volumetric_drilling_msgs

#endif  // VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__VOXELS__BUILDER_HPP_

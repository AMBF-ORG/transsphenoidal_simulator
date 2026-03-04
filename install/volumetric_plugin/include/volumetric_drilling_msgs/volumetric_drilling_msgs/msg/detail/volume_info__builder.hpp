// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from volumetric_drilling_msgs:msg/VolumeInfo.idl
// generated code does not contain a copyright notice

#ifndef VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__VOLUME_INFO__BUILDER_HPP_
#define VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__VOLUME_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "volumetric_drilling_msgs/msg/detail/volume_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace volumetric_drilling_msgs
{

namespace msg
{

namespace builder
{

class Init_VolumeInfo_voxel_count
{
public:
  explicit Init_VolumeInfo_voxel_count(::volumetric_drilling_msgs::msg::VolumeInfo & msg)
  : msg_(msg)
  {}
  ::volumetric_drilling_msgs::msg::VolumeInfo voxel_count(::volumetric_drilling_msgs::msg::VolumeInfo::_voxel_count_type arg)
  {
    msg_.voxel_count = std::move(arg);
    return std::move(msg_);
  }

private:
  ::volumetric_drilling_msgs::msg::VolumeInfo msg_;
};

class Init_VolumeInfo_dimensions
{
public:
  explicit Init_VolumeInfo_dimensions(::volumetric_drilling_msgs::msg::VolumeInfo & msg)
  : msg_(msg)
  {}
  Init_VolumeInfo_voxel_count dimensions(::volumetric_drilling_msgs::msg::VolumeInfo::_dimensions_type arg)
  {
    msg_.dimensions = std::move(arg);
    return Init_VolumeInfo_voxel_count(msg_);
  }

private:
  ::volumetric_drilling_msgs::msg::VolumeInfo msg_;
};

class Init_VolumeInfo_pose
{
public:
  explicit Init_VolumeInfo_pose(::volumetric_drilling_msgs::msg::VolumeInfo & msg)
  : msg_(msg)
  {}
  Init_VolumeInfo_dimensions pose(::volumetric_drilling_msgs::msg::VolumeInfo::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_VolumeInfo_dimensions(msg_);
  }

private:
  ::volumetric_drilling_msgs::msg::VolumeInfo msg_;
};

class Init_VolumeInfo_header
{
public:
  Init_VolumeInfo_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VolumeInfo_pose header(::volumetric_drilling_msgs::msg::VolumeInfo::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_VolumeInfo_pose(msg_);
  }

private:
  ::volumetric_drilling_msgs::msg::VolumeInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::volumetric_drilling_msgs::msg::VolumeInfo>()
{
  return volumetric_drilling_msgs::msg::builder::Init_VolumeInfo_header();
}

}  // namespace volumetric_drilling_msgs

#endif  // VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__VOLUME_INFO__BUILDER_HPP_

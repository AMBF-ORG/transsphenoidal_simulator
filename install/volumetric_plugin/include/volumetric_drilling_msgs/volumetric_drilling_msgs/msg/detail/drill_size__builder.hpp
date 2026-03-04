// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from volumetric_drilling_msgs:msg/DrillSize.idl
// generated code does not contain a copyright notice

#ifndef VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__DRILL_SIZE__BUILDER_HPP_
#define VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__DRILL_SIZE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "volumetric_drilling_msgs/msg/detail/drill_size__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace volumetric_drilling_msgs
{

namespace msg
{

namespace builder
{

class Init_DrillSize_size
{
public:
  explicit Init_DrillSize_size(::volumetric_drilling_msgs::msg::DrillSize & msg)
  : msg_(msg)
  {}
  ::volumetric_drilling_msgs::msg::DrillSize size(::volumetric_drilling_msgs::msg::DrillSize::_size_type arg)
  {
    msg_.size = std::move(arg);
    return std::move(msg_);
  }

private:
  ::volumetric_drilling_msgs::msg::DrillSize msg_;
};

class Init_DrillSize_header
{
public:
  Init_DrillSize_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DrillSize_size header(::volumetric_drilling_msgs::msg::DrillSize::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DrillSize_size(msg_);
  }

private:
  ::volumetric_drilling_msgs::msg::DrillSize msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::volumetric_drilling_msgs::msg::DrillSize>()
{
  return volumetric_drilling_msgs::msg::builder::Init_DrillSize_header();
}

}  // namespace volumetric_drilling_msgs

#endif  // VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__DRILL_SIZE__BUILDER_HPP_

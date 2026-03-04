// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from volumetric_drilling_msgs:msg/Index.idl
// generated code does not contain a copyright notice

#ifndef VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__INDEX__BUILDER_HPP_
#define VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__INDEX__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "volumetric_drilling_msgs/msg/detail/index__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace volumetric_drilling_msgs
{

namespace msg
{

namespace builder
{

class Init_Index_z
{
public:
  explicit Init_Index_z(::volumetric_drilling_msgs::msg::Index & msg)
  : msg_(msg)
  {}
  ::volumetric_drilling_msgs::msg::Index z(::volumetric_drilling_msgs::msg::Index::_z_type arg)
  {
    msg_.z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::volumetric_drilling_msgs::msg::Index msg_;
};

class Init_Index_y
{
public:
  explicit Init_Index_y(::volumetric_drilling_msgs::msg::Index & msg)
  : msg_(msg)
  {}
  Init_Index_z y(::volumetric_drilling_msgs::msg::Index::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Index_z(msg_);
  }

private:
  ::volumetric_drilling_msgs::msg::Index msg_;
};

class Init_Index_x
{
public:
  Init_Index_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Index_y x(::volumetric_drilling_msgs::msg::Index::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Index_y(msg_);
  }

private:
  ::volumetric_drilling_msgs::msg::Index msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::volumetric_drilling_msgs::msg::Index>()
{
  return volumetric_drilling_msgs::msg::builder::Init_Index_x();
}

}  // namespace volumetric_drilling_msgs

#endif  // VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__INDEX__BUILDER_HPP_

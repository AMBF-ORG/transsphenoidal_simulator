// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from volumetric_drilling_msgs:msg/DrillSize.idl
// generated code does not contain a copyright notice

#ifndef VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__DRILL_SIZE__STRUCT_HPP_
#define VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__DRILL_SIZE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'size'
#include "std_msgs/msg/detail/u_int8__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__volumetric_drilling_msgs__msg__DrillSize __attribute__((deprecated))
#else
# define DEPRECATED__volumetric_drilling_msgs__msg__DrillSize __declspec(deprecated)
#endif

namespace volumetric_drilling_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DrillSize_
{
  using Type = DrillSize_<ContainerAllocator>;

  explicit DrillSize_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    size(_init)
  {
    (void)_init;
  }

  explicit DrillSize_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    size(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _size_type =
    std_msgs::msg::UInt8_<ContainerAllocator>;
  _size_type size;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__size(
    const std_msgs::msg::UInt8_<ContainerAllocator> & _arg)
  {
    this->size = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    volumetric_drilling_msgs::msg::DrillSize_<ContainerAllocator> *;
  using ConstRawPtr =
    const volumetric_drilling_msgs::msg::DrillSize_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<volumetric_drilling_msgs::msg::DrillSize_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<volumetric_drilling_msgs::msg::DrillSize_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      volumetric_drilling_msgs::msg::DrillSize_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<volumetric_drilling_msgs::msg::DrillSize_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      volumetric_drilling_msgs::msg::DrillSize_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<volumetric_drilling_msgs::msg::DrillSize_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<volumetric_drilling_msgs::msg::DrillSize_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<volumetric_drilling_msgs::msg::DrillSize_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__volumetric_drilling_msgs__msg__DrillSize
    std::shared_ptr<volumetric_drilling_msgs::msg::DrillSize_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__volumetric_drilling_msgs__msg__DrillSize
    std::shared_ptr<volumetric_drilling_msgs::msg::DrillSize_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DrillSize_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->size != other.size) {
      return false;
    }
    return true;
  }
  bool operator!=(const DrillSize_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DrillSize_

// alias to use template instance with default allocator
using DrillSize =
  volumetric_drilling_msgs::msg::DrillSize_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace volumetric_drilling_msgs

#endif  // VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__DRILL_SIZE__STRUCT_HPP_

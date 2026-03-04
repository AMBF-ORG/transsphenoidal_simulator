// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from volumetric_drilling_msgs:msg/Index.idl
// generated code does not contain a copyright notice

#ifndef VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__INDEX__STRUCT_HPP_
#define VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__INDEX__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__volumetric_drilling_msgs__msg__Index __attribute__((deprecated))
#else
# define DEPRECATED__volumetric_drilling_msgs__msg__Index __declspec(deprecated)
#endif

namespace volumetric_drilling_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Index_
{
  using Type = Index_<ContainerAllocator>;

  explicit Index_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0ll;
      this->y = 0ll;
      this->z = 0ll;
    }
  }

  explicit Index_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0ll;
      this->y = 0ll;
      this->z = 0ll;
    }
  }

  // field types and members
  using _x_type =
    int64_t;
  _x_type x;
  using _y_type =
    int64_t;
  _y_type y;
  using _z_type =
    int64_t;
  _z_type z;

  // setters for named parameter idiom
  Type & set__x(
    const int64_t & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const int64_t & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const int64_t & _arg)
  {
    this->z = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    volumetric_drilling_msgs::msg::Index_<ContainerAllocator> *;
  using ConstRawPtr =
    const volumetric_drilling_msgs::msg::Index_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<volumetric_drilling_msgs::msg::Index_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<volumetric_drilling_msgs::msg::Index_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      volumetric_drilling_msgs::msg::Index_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<volumetric_drilling_msgs::msg::Index_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      volumetric_drilling_msgs::msg::Index_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<volumetric_drilling_msgs::msg::Index_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<volumetric_drilling_msgs::msg::Index_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<volumetric_drilling_msgs::msg::Index_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__volumetric_drilling_msgs__msg__Index
    std::shared_ptr<volumetric_drilling_msgs::msg::Index_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__volumetric_drilling_msgs__msg__Index
    std::shared_ptr<volumetric_drilling_msgs::msg::Index_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Index_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    return true;
  }
  bool operator!=(const Index_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Index_

// alias to use template instance with default allocator
using Index =
  volumetric_drilling_msgs::msg::Index_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace volumetric_drilling_msgs

#endif  // VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__INDEX__STRUCT_HPP_

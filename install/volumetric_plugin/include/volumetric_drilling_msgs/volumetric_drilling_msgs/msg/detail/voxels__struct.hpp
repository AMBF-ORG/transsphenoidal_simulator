// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from volumetric_drilling_msgs:msg/Voxels.idl
// generated code does not contain a copyright notice

#ifndef VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__VOXELS__STRUCT_HPP_
#define VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__VOXELS__STRUCT_HPP_

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
// Member 'indices'
#include "volumetric_drilling_msgs/msg/detail/index__struct.hpp"
// Member 'colors'
#include "std_msgs/msg/detail/color_rgba__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__volumetric_drilling_msgs__msg__Voxels __attribute__((deprecated))
#else
# define DEPRECATED__volumetric_drilling_msgs__msg__Voxels __declspec(deprecated)
#endif

namespace volumetric_drilling_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Voxels_
{
  using Type = Voxels_<ContainerAllocator>;

  explicit Voxels_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit Voxels_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _indices_type =
    std::vector<volumetric_drilling_msgs::msg::Index_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<volumetric_drilling_msgs::msg::Index_<ContainerAllocator>>>;
  _indices_type indices;
  using _colors_type =
    std::vector<std_msgs::msg::ColorRGBA_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std_msgs::msg::ColorRGBA_<ContainerAllocator>>>;
  _colors_type colors;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__indices(
    const std::vector<volumetric_drilling_msgs::msg::Index_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<volumetric_drilling_msgs::msg::Index_<ContainerAllocator>>> & _arg)
  {
    this->indices = _arg;
    return *this;
  }
  Type & set__colors(
    const std::vector<std_msgs::msg::ColorRGBA_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std_msgs::msg::ColorRGBA_<ContainerAllocator>>> & _arg)
  {
    this->colors = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    volumetric_drilling_msgs::msg::Voxels_<ContainerAllocator> *;
  using ConstRawPtr =
    const volumetric_drilling_msgs::msg::Voxels_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<volumetric_drilling_msgs::msg::Voxels_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<volumetric_drilling_msgs::msg::Voxels_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      volumetric_drilling_msgs::msg::Voxels_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<volumetric_drilling_msgs::msg::Voxels_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      volumetric_drilling_msgs::msg::Voxels_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<volumetric_drilling_msgs::msg::Voxels_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<volumetric_drilling_msgs::msg::Voxels_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<volumetric_drilling_msgs::msg::Voxels_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__volumetric_drilling_msgs__msg__Voxels
    std::shared_ptr<volumetric_drilling_msgs::msg::Voxels_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__volumetric_drilling_msgs__msg__Voxels
    std::shared_ptr<volumetric_drilling_msgs::msg::Voxels_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Voxels_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->indices != other.indices) {
      return false;
    }
    if (this->colors != other.colors) {
      return false;
    }
    return true;
  }
  bool operator!=(const Voxels_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Voxels_

// alias to use template instance with default allocator
using Voxels =
  volumetric_drilling_msgs::msg::Voxels_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace volumetric_drilling_msgs

#endif  // VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__VOXELS__STRUCT_HPP_

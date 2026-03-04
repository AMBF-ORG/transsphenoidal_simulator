// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from volumetric_drilling_msgs:msg/VolumeInfo.idl
// generated code does not contain a copyright notice

#ifndef VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__VOLUME_INFO__STRUCT_HPP_
#define VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__VOLUME_INFO__STRUCT_HPP_

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
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__volumetric_drilling_msgs__msg__VolumeInfo __attribute__((deprecated))
#else
# define DEPRECATED__volumetric_drilling_msgs__msg__VolumeInfo __declspec(deprecated)
#endif

namespace volumetric_drilling_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VolumeInfo_
{
  using Type = VolumeInfo_<ContainerAllocator>;

  explicit VolumeInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pose(_init)
  {
    (void)_init;
  }

  explicit VolumeInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    pose(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _dimensions_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _dimensions_type dimensions;
  using _voxel_count_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _voxel_count_type voxel_count;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__dimensions(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->dimensions = _arg;
    return *this;
  }
  Type & set__voxel_count(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->voxel_count = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    volumetric_drilling_msgs::msg::VolumeInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const volumetric_drilling_msgs::msg::VolumeInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<volumetric_drilling_msgs::msg::VolumeInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<volumetric_drilling_msgs::msg::VolumeInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      volumetric_drilling_msgs::msg::VolumeInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<volumetric_drilling_msgs::msg::VolumeInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      volumetric_drilling_msgs::msg::VolumeInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<volumetric_drilling_msgs::msg::VolumeInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<volumetric_drilling_msgs::msg::VolumeInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<volumetric_drilling_msgs::msg::VolumeInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__volumetric_drilling_msgs__msg__VolumeInfo
    std::shared_ptr<volumetric_drilling_msgs::msg::VolumeInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__volumetric_drilling_msgs__msg__VolumeInfo
    std::shared_ptr<volumetric_drilling_msgs::msg::VolumeInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VolumeInfo_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->dimensions != other.dimensions) {
      return false;
    }
    if (this->voxel_count != other.voxel_count) {
      return false;
    }
    return true;
  }
  bool operator!=(const VolumeInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VolumeInfo_

// alias to use template instance with default allocator
using VolumeInfo =
  volumetric_drilling_msgs::msg::VolumeInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace volumetric_drilling_msgs

#endif  // VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__VOLUME_INFO__STRUCT_HPP_

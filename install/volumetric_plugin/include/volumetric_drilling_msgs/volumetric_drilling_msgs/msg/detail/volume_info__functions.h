// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from volumetric_drilling_msgs:msg/VolumeInfo.idl
// generated code does not contain a copyright notice

#ifndef VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__VOLUME_INFO__FUNCTIONS_H_
#define VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__VOLUME_INFO__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "volumetric_drilling_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "volumetric_drilling_msgs/msg/detail/volume_info__struct.h"

/// Initialize msg/VolumeInfo message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * volumetric_drilling_msgs__msg__VolumeInfo
 * )) before or use
 * volumetric_drilling_msgs__msg__VolumeInfo__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_volumetric_drilling_msgs
bool
volumetric_drilling_msgs__msg__VolumeInfo__init(volumetric_drilling_msgs__msg__VolumeInfo * msg);

/// Finalize msg/VolumeInfo message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_volumetric_drilling_msgs
void
volumetric_drilling_msgs__msg__VolumeInfo__fini(volumetric_drilling_msgs__msg__VolumeInfo * msg);

/// Create msg/VolumeInfo message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * volumetric_drilling_msgs__msg__VolumeInfo__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_volumetric_drilling_msgs
volumetric_drilling_msgs__msg__VolumeInfo *
volumetric_drilling_msgs__msg__VolumeInfo__create();

/// Destroy msg/VolumeInfo message.
/**
 * It calls
 * volumetric_drilling_msgs__msg__VolumeInfo__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_volumetric_drilling_msgs
void
volumetric_drilling_msgs__msg__VolumeInfo__destroy(volumetric_drilling_msgs__msg__VolumeInfo * msg);

/// Check for msg/VolumeInfo message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_volumetric_drilling_msgs
bool
volumetric_drilling_msgs__msg__VolumeInfo__are_equal(const volumetric_drilling_msgs__msg__VolumeInfo * lhs, const volumetric_drilling_msgs__msg__VolumeInfo * rhs);

/// Copy a msg/VolumeInfo message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_volumetric_drilling_msgs
bool
volumetric_drilling_msgs__msg__VolumeInfo__copy(
  const volumetric_drilling_msgs__msg__VolumeInfo * input,
  volumetric_drilling_msgs__msg__VolumeInfo * output);

/// Initialize array of msg/VolumeInfo messages.
/**
 * It allocates the memory for the number of elements and calls
 * volumetric_drilling_msgs__msg__VolumeInfo__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_volumetric_drilling_msgs
bool
volumetric_drilling_msgs__msg__VolumeInfo__Sequence__init(volumetric_drilling_msgs__msg__VolumeInfo__Sequence * array, size_t size);

/// Finalize array of msg/VolumeInfo messages.
/**
 * It calls
 * volumetric_drilling_msgs__msg__VolumeInfo__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_volumetric_drilling_msgs
void
volumetric_drilling_msgs__msg__VolumeInfo__Sequence__fini(volumetric_drilling_msgs__msg__VolumeInfo__Sequence * array);

/// Create array of msg/VolumeInfo messages.
/**
 * It allocates the memory for the array and calls
 * volumetric_drilling_msgs__msg__VolumeInfo__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_volumetric_drilling_msgs
volumetric_drilling_msgs__msg__VolumeInfo__Sequence *
volumetric_drilling_msgs__msg__VolumeInfo__Sequence__create(size_t size);

/// Destroy array of msg/VolumeInfo messages.
/**
 * It calls
 * volumetric_drilling_msgs__msg__VolumeInfo__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_volumetric_drilling_msgs
void
volumetric_drilling_msgs__msg__VolumeInfo__Sequence__destroy(volumetric_drilling_msgs__msg__VolumeInfo__Sequence * array);

/// Check for msg/VolumeInfo message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_volumetric_drilling_msgs
bool
volumetric_drilling_msgs__msg__VolumeInfo__Sequence__are_equal(const volumetric_drilling_msgs__msg__VolumeInfo__Sequence * lhs, const volumetric_drilling_msgs__msg__VolumeInfo__Sequence * rhs);

/// Copy an array of msg/VolumeInfo messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_volumetric_drilling_msgs
bool
volumetric_drilling_msgs__msg__VolumeInfo__Sequence__copy(
  const volumetric_drilling_msgs__msg__VolumeInfo__Sequence * input,
  volumetric_drilling_msgs__msg__VolumeInfo__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__VOLUME_INFO__FUNCTIONS_H_

// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from volumetric_drilling_msgs:msg/Voxels.idl
// generated code does not contain a copyright notice

#ifndef VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__VOXELS__FUNCTIONS_H_
#define VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__VOXELS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "volumetric_drilling_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "volumetric_drilling_msgs/msg/detail/voxels__struct.h"

/// Initialize msg/Voxels message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * volumetric_drilling_msgs__msg__Voxels
 * )) before or use
 * volumetric_drilling_msgs__msg__Voxels__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_volumetric_drilling_msgs
bool
volumetric_drilling_msgs__msg__Voxels__init(volumetric_drilling_msgs__msg__Voxels * msg);

/// Finalize msg/Voxels message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_volumetric_drilling_msgs
void
volumetric_drilling_msgs__msg__Voxels__fini(volumetric_drilling_msgs__msg__Voxels * msg);

/// Create msg/Voxels message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * volumetric_drilling_msgs__msg__Voxels__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_volumetric_drilling_msgs
volumetric_drilling_msgs__msg__Voxels *
volumetric_drilling_msgs__msg__Voxels__create();

/// Destroy msg/Voxels message.
/**
 * It calls
 * volumetric_drilling_msgs__msg__Voxels__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_volumetric_drilling_msgs
void
volumetric_drilling_msgs__msg__Voxels__destroy(volumetric_drilling_msgs__msg__Voxels * msg);

/// Check for msg/Voxels message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_volumetric_drilling_msgs
bool
volumetric_drilling_msgs__msg__Voxels__are_equal(const volumetric_drilling_msgs__msg__Voxels * lhs, const volumetric_drilling_msgs__msg__Voxels * rhs);

/// Copy a msg/Voxels message.
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
volumetric_drilling_msgs__msg__Voxels__copy(
  const volumetric_drilling_msgs__msg__Voxels * input,
  volumetric_drilling_msgs__msg__Voxels * output);

/// Initialize array of msg/Voxels messages.
/**
 * It allocates the memory for the number of elements and calls
 * volumetric_drilling_msgs__msg__Voxels__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_volumetric_drilling_msgs
bool
volumetric_drilling_msgs__msg__Voxels__Sequence__init(volumetric_drilling_msgs__msg__Voxels__Sequence * array, size_t size);

/// Finalize array of msg/Voxels messages.
/**
 * It calls
 * volumetric_drilling_msgs__msg__Voxels__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_volumetric_drilling_msgs
void
volumetric_drilling_msgs__msg__Voxels__Sequence__fini(volumetric_drilling_msgs__msg__Voxels__Sequence * array);

/// Create array of msg/Voxels messages.
/**
 * It allocates the memory for the array and calls
 * volumetric_drilling_msgs__msg__Voxels__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_volumetric_drilling_msgs
volumetric_drilling_msgs__msg__Voxels__Sequence *
volumetric_drilling_msgs__msg__Voxels__Sequence__create(size_t size);

/// Destroy array of msg/Voxels messages.
/**
 * It calls
 * volumetric_drilling_msgs__msg__Voxels__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_volumetric_drilling_msgs
void
volumetric_drilling_msgs__msg__Voxels__Sequence__destroy(volumetric_drilling_msgs__msg__Voxels__Sequence * array);

/// Check for msg/Voxels message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_volumetric_drilling_msgs
bool
volumetric_drilling_msgs__msg__Voxels__Sequence__are_equal(const volumetric_drilling_msgs__msg__Voxels__Sequence * lhs, const volumetric_drilling_msgs__msg__Voxels__Sequence * rhs);

/// Copy an array of msg/Voxels messages.
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
volumetric_drilling_msgs__msg__Voxels__Sequence__copy(
  const volumetric_drilling_msgs__msg__Voxels__Sequence * input,
  volumetric_drilling_msgs__msg__Voxels__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // VOLUMETRIC_DRILLING_MSGS__MSG__DETAIL__VOXELS__FUNCTIONS_H_

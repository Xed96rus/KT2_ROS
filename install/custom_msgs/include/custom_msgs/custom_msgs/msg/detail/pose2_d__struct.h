// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:msg/Pose2D.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__POSE2_D__STRUCT_H_
#define CUSTOM_MSGS__MSG__DETAIL__POSE2_D__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Pose2D in the package custom_msgs.
/**
  * Файл: custom_msgs/msg/Pose2D.msg
 */
typedef struct custom_msgs__msg__Pose2D
{
  float x;
  float y;
  float theta;
} custom_msgs__msg__Pose2D;

// Struct for a sequence of custom_msgs__msg__Pose2D.
typedef struct custom_msgs__msg__Pose2D__Sequence
{
  custom_msgs__msg__Pose2D * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__msg__Pose2D__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__POSE2_D__STRUCT_H_

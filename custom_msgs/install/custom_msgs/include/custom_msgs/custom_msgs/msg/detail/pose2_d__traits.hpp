// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_msgs:msg/Pose2D.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__POSE2_D__TRAITS_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__POSE2_D__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_msgs/msg/detail/pose2_d__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace custom_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Pose2D & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: theta
  {
    out << "theta: ";
    rosidl_generator_traits::value_to_yaml(msg.theta, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Pose2D & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: theta
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "theta: ";
    rosidl_generator_traits::value_to_yaml(msg.theta, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Pose2D & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace custom_msgs

namespace rosidl_generator_traits
{

[[deprecated("use custom_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_msgs::msg::Pose2D & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_msgs::msg::Pose2D & msg)
{
  return custom_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_msgs::msg::Pose2D>()
{
  return "custom_msgs::msg::Pose2D";
}

template<>
inline const char * name<custom_msgs::msg::Pose2D>()
{
  return "custom_msgs/msg/Pose2D";
}

template<>
struct has_fixed_size<custom_msgs::msg::Pose2D>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<custom_msgs::msg::Pose2D>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<custom_msgs::msg::Pose2D>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_MSGS__MSG__DETAIL__POSE2_D__TRAITS_HPP_

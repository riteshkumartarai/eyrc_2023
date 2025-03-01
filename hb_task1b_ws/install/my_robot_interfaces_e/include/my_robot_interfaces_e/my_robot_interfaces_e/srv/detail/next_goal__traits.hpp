// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from my_robot_interfaces_e:srv/NextGoal.idl
// generated code does not contain a copyright notice

#ifndef MY_ROBOT_INTERFACES_E__SRV__DETAIL__NEXT_GOAL__TRAITS_HPP_
#define MY_ROBOT_INTERFACES_E__SRV__DETAIL__NEXT_GOAL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "my_robot_interfaces_e/srv/detail/next_goal__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace my_robot_interfaces_e
{

namespace srv
{

inline void to_flow_style_yaml(
  const NextGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: request_goal
  {
    out << "request_goal: ";
    rosidl_generator_traits::value_to_yaml(msg.request_goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NextGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: request_goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "request_goal: ";
    rosidl_generator_traits::value_to_yaml(msg.request_goal, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NextGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace my_robot_interfaces_e

namespace rosidl_generator_traits
{

[[deprecated("use my_robot_interfaces_e::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const my_robot_interfaces_e::srv::NextGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  my_robot_interfaces_e::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use my_robot_interfaces_e::srv::to_yaml() instead")]]
inline std::string to_yaml(const my_robot_interfaces_e::srv::NextGoal_Request & msg)
{
  return my_robot_interfaces_e::srv::to_yaml(msg);
}

template<>
inline const char * data_type<my_robot_interfaces_e::srv::NextGoal_Request>()
{
  return "my_robot_interfaces_e::srv::NextGoal_Request";
}

template<>
inline const char * name<my_robot_interfaces_e::srv::NextGoal_Request>()
{
  return "my_robot_interfaces_e/srv/NextGoal_Request";
}

template<>
struct has_fixed_size<my_robot_interfaces_e::srv::NextGoal_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<my_robot_interfaces_e::srv::NextGoal_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<my_robot_interfaces_e::srv::NextGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace my_robot_interfaces_e
{

namespace srv
{

inline void to_flow_style_yaml(
  const NextGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: x_goal
  {
    out << "x_goal: ";
    rosidl_generator_traits::value_to_yaml(msg.x_goal, out);
    out << ", ";
  }

  // member: y_goal
  {
    out << "y_goal: ";
    rosidl_generator_traits::value_to_yaml(msg.y_goal, out);
    out << ", ";
  }

  // member: theta_goal
  {
    out << "theta_goal: ";
    rosidl_generator_traits::value_to_yaml(msg.theta_goal, out);
    out << ", ";
  }

  // member: end_of_list
  {
    out << "end_of_list: ";
    rosidl_generator_traits::value_to_yaml(msg.end_of_list, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NextGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x_goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x_goal: ";
    rosidl_generator_traits::value_to_yaml(msg.x_goal, out);
    out << "\n";
  }

  // member: y_goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y_goal: ";
    rosidl_generator_traits::value_to_yaml(msg.y_goal, out);
    out << "\n";
  }

  // member: theta_goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "theta_goal: ";
    rosidl_generator_traits::value_to_yaml(msg.theta_goal, out);
    out << "\n";
  }

  // member: end_of_list
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "end_of_list: ";
    rosidl_generator_traits::value_to_yaml(msg.end_of_list, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NextGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace my_robot_interfaces_e

namespace rosidl_generator_traits
{

[[deprecated("use my_robot_interfaces_e::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const my_robot_interfaces_e::srv::NextGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  my_robot_interfaces_e::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use my_robot_interfaces_e::srv::to_yaml() instead")]]
inline std::string to_yaml(const my_robot_interfaces_e::srv::NextGoal_Response & msg)
{
  return my_robot_interfaces_e::srv::to_yaml(msg);
}

template<>
inline const char * data_type<my_robot_interfaces_e::srv::NextGoal_Response>()
{
  return "my_robot_interfaces_e::srv::NextGoal_Response";
}

template<>
inline const char * name<my_robot_interfaces_e::srv::NextGoal_Response>()
{
  return "my_robot_interfaces_e/srv/NextGoal_Response";
}

template<>
struct has_fixed_size<my_robot_interfaces_e::srv::NextGoal_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<my_robot_interfaces_e::srv::NextGoal_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<my_robot_interfaces_e::srv::NextGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<my_robot_interfaces_e::srv::NextGoal>()
{
  return "my_robot_interfaces_e::srv::NextGoal";
}

template<>
inline const char * name<my_robot_interfaces_e::srv::NextGoal>()
{
  return "my_robot_interfaces_e/srv/NextGoal";
}

template<>
struct has_fixed_size<my_robot_interfaces_e::srv::NextGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<my_robot_interfaces_e::srv::NextGoal_Request>::value &&
    has_fixed_size<my_robot_interfaces_e::srv::NextGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<my_robot_interfaces_e::srv::NextGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<my_robot_interfaces_e::srv::NextGoal_Request>::value &&
    has_bounded_size<my_robot_interfaces_e::srv::NextGoal_Response>::value
  >
{
};

template<>
struct is_service<my_robot_interfaces_e::srv::NextGoal>
  : std::true_type
{
};

template<>
struct is_service_request<my_robot_interfaces_e::srv::NextGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<my_robot_interfaces_e::srv::NextGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MY_ROBOT_INTERFACES_E__SRV__DETAIL__NEXT_GOAL__TRAITS_HPP_

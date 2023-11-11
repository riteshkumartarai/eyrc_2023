// generated from rosidl_generator_c/resource/rosidl_generator_c__visibility_control.h.in
// generated code does not contain a copyright notice

#ifndef MY_ROBOT_INTERFACES_E__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_
#define MY_ROBOT_INTERFACES_E__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_GENERATOR_C_EXPORT_my_robot_interfaces_e __attribute__ ((dllexport))
    #define ROSIDL_GENERATOR_C_IMPORT_my_robot_interfaces_e __attribute__ ((dllimport))
  #else
    #define ROSIDL_GENERATOR_C_EXPORT_my_robot_interfaces_e __declspec(dllexport)
    #define ROSIDL_GENERATOR_C_IMPORT_my_robot_interfaces_e __declspec(dllimport)
  #endif
  #ifdef ROSIDL_GENERATOR_C_BUILDING_DLL_my_robot_interfaces_e
    #define ROSIDL_GENERATOR_C_PUBLIC_my_robot_interfaces_e ROSIDL_GENERATOR_C_EXPORT_my_robot_interfaces_e
  #else
    #define ROSIDL_GENERATOR_C_PUBLIC_my_robot_interfaces_e ROSIDL_GENERATOR_C_IMPORT_my_robot_interfaces_e
  #endif
#else
  #define ROSIDL_GENERATOR_C_EXPORT_my_robot_interfaces_e __attribute__ ((visibility("default")))
  #define ROSIDL_GENERATOR_C_IMPORT_my_robot_interfaces_e
  #if __GNUC__ >= 4
    #define ROSIDL_GENERATOR_C_PUBLIC_my_robot_interfaces_e __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_GENERATOR_C_PUBLIC_my_robot_interfaces_e
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // MY_ROBOT_INTERFACES_E__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_

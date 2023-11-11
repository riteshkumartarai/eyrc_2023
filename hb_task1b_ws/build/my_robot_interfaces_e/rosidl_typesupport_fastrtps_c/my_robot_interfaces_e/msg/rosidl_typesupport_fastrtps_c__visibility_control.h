// generated from
// rosidl_typesupport_fastrtps_c/resource/rosidl_typesupport_fastrtps_c__visibility_control.h.in
// generated code does not contain a copyright notice

#ifndef MY_ROBOT_INTERFACES_E__MSG__ROSIDL_TYPESUPPORT_FASTRTPS_C__VISIBILITY_CONTROL_H_
#define MY_ROBOT_INTERFACES_E__MSG__ROSIDL_TYPESUPPORT_FASTRTPS_C__VISIBILITY_CONTROL_H_

#if __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_EXPORT_my_robot_interfaces_e __attribute__ ((dllexport))
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_my_robot_interfaces_e __attribute__ ((dllimport))
  #else
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_EXPORT_my_robot_interfaces_e __declspec(dllexport)
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_my_robot_interfaces_e __declspec(dllimport)
  #endif
  #ifdef ROSIDL_TYPESUPPORT_FASTRTPS_C_BUILDING_DLL_my_robot_interfaces_e
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_my_robot_interfaces_e ROSIDL_TYPESUPPORT_FASTRTPS_C_EXPORT_my_robot_interfaces_e
  #else
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_my_robot_interfaces_e ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_my_robot_interfaces_e
  #endif
#else
  #define ROSIDL_TYPESUPPORT_FASTRTPS_C_EXPORT_my_robot_interfaces_e __attribute__ ((visibility("default")))
  #define ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_my_robot_interfaces_e
  #if __GNUC__ >= 4
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_my_robot_interfaces_e __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_my_robot_interfaces_e
  #endif
#endif

#if __cplusplus
}
#endif

#endif  // MY_ROBOT_INTERFACES_E__MSG__ROSIDL_TYPESUPPORT_FASTRTPS_C__VISIBILITY_CONTROL_H_

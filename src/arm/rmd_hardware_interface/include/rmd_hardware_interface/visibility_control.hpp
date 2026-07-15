/**
 * \file visibility_control.hpp
 * \mainpage
 *    Auto-generated C++ header by "ros2 pkg create"
 *    This logic was borrowed (then namespaced) from the examples on the GCC wiki:
 *    https://gcc.gnu.org/wiki/Visibility
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef RMD_HARDWARE_INTERFACE__VISIBILITY_CONTROL
#define RMD_HARDWARE_INTERFACE__VISIBILITY_CONTROL
#pragma once

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RMD_HARDWARE_INTERFACE_EXPORT __attribute__ ((dllexport))
    #define RMD_HARDWARE_INTERFACE_IMPORT __attribute__ ((dllimport))
  #else
    #define RMD_HARDWARE_INTERFACE_EXPORT __declspec(dllexport)
    #define RMD_HARDWARE_INTERFACE_IMPORT __declspec(dllimport)
  #endif
  #ifdef RMD_HARDWARE_INTERFACE_BUILDING_LIBRARY
    #define RMD_HARDWARE_INTERFACE_PUBLIC RMD_HARDWARE_INTERFACE_EXPORT
  #else
    #define RMD_HARDWARE_INTERFACE_PUBLIC RMD_HARDWARE_INTERFACE_IMPORT
  #endif
  #define RMD_HARDWARE_INTERFACE_PUBLIC_TYPE RMD_HARDWARE_INTERFACE_PUBLIC
  #define RMD_HARDWARE_INTERFACE_LOCAL
#else
  #define RMD_HARDWARE_INTERFACE_EXPORT __attribute__ ((visibility("default")))
  #define RMD_HARDWARE_INTERFACE_IMPORT
  #if __GNUC__ >= 4
    #define RMD_HARDWARE_INTERFACE_PUBLIC __attribute__ ((visibility("default")))
    #define RMD_HARDWARE_INTERFACE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RMD_HARDWARE_INTERFACE_PUBLIC
    #define RMD_HARDWARE_INTERFACE_LOCAL
  #endif
  #define RMD_HARDWARE_INTERFACE_PUBLIC_TYPE
#endif

#endif  // RMD_HARDWARE_INTERFACE__VISIBILITY_CONTROL

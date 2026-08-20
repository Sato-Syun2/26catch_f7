#ifndef CATCH26_INTERFACE__VISIBILITY_CONTROL_H_
#define CATCH26_INTERFACE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CATCH26_INTERFACE_EXPORT __attribute__ ((dllexport))
    #define CATCH26_INTERFACE_IMPORT __attribute__ ((dllimport))
  #else
    #define CATCH26_INTERFACE_EXPORT __declspec(dllexport)
    #define CATCH26_INTERFACE_IMPORT __declspec(dllimport)
  #endif
  #ifdef CATCH26_INTERFACE_BUILDING_LIBRARY
    #define CATCH26_INTERFACE_PUBLIC CATCH26_INTERFACE_EXPORT
  #else
    #define CATCH26_INTERFACE_PUBLIC CATCH26_INTERFACE_IMPORT
  #endif
  #define CATCH26_INTERFACE_PUBLIC_TYPE CATCH26_INTERFACE_PUBLIC
  #define CATCH26_INTERFACE_LOCAL
#else
  #define CATCH26_INTERFACE_EXPORT __attribute__ ((visibility("default")))
  #define CATCH26_INTERFACE_IMPORT
  #if __GNUC__ >= 4
    #define CATCH26_INTERFACE_PUBLIC __attribute__ ((visibility("default")))
    #define CATCH26_INTERFACE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CATCH26_INTERFACE_PUBLIC
    #define CATCH26_INTERFACE_LOCAL
  #endif
  #define CATCH26_INTERFACE_PUBLIC_TYPE
#endif

#endif  // CATCH26_INTERFACE__VISIBILITY_CONTROL_H_

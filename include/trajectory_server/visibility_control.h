#ifndef TRAJECTORY_SERVER__VISIBILITY_CONTROL_H_
#define TRAJECTORY_SERVER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TRAJECTORY_SERVER_EXPORT __attribute__ ((dllexport))
    #define TRAJECTORY_SERVER_IMPORT __attribute__ ((dllimport))
  #else
    #define TRAJECTORY_SERVER_EXPORT __declspec(dllexport)
    #define TRAJECTORY_SERVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef TRAJECTORY_SERVER_BUILDING_LIBRARY
    #define TRAJECTORY_SERVER_PUBLIC TRAJECTORY_SERVER_EXPORT
  #else
    #define TRAJECTORY_SERVER_PUBLIC TRAJECTORY_SERVER_IMPORT
  #endif
  #define TRAJECTORY_SERVER_PUBLIC_TYPE TRAJECTORY_SERVER_PUBLIC
  #define TRAJECTORY_SERVER_LOCAL
#else
  #define TRAJECTORY_SERVER_EXPORT __attribute__ ((visibility("default")))
  #define TRAJECTORY_SERVER_IMPORT
  #if __GNUC__ >= 4
    #define TRAJECTORY_SERVER_PUBLIC __attribute__ ((visibility("default")))
    #define TRAJECTORY_SERVER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TRAJECTORY_SERVER_PUBLIC
    #define TRAJECTORY_SERVER_LOCAL
  #endif
  #define TRAJECTORY_SERVER_PUBLIC_TYPE
#endif

#endif  // TRAJECTORY_SERVER__VISIBILITY_CONTROL_H_

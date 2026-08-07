# PhysX Custom Build Configuration Types
#
# Single source of truth for PhysX's custom build configurations:
# debug, checked, profile, release (instead of CMake's standard Debug, Release, etc.)
#
# This module is included from both:
#   - physx/CMakeLists.txt (FetchContent entry point, before targets)
#   - physx/source/compiler/cmake/CMakeLists.txt (standalone build)
#
# It must be called AFTER project() so that MSVC is set, and BEFORE any target is created.

if(CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_CONFIGURATION_TYPES debug checked profile release CACHE STRING
      "Reset config to what we need" FORCE)

  # CMake doesn't automatically create linker-flag variables for non-standard
  # configurations, which causes "Missing variable" errors during generation.
  foreach(_config DEBUG CHECKED PROFILE RELEASE debug checked profile release)
    foreach(_linker_type EXE SHARED STATIC)
      set(CMAKE_${_linker_type}_LINKER_FLAGS_${_config} "" CACHE STRING "" FORCE)
    endforeach()
  endforeach()

  # CMake only sets _DEBUG/NDEBUG automatically for standard configs (Debug/Release).
  # For custom configs we must set them explicitly.
  if(MSVC)
    set(_msvc_debug_flags "/Zi /Ob0 /Od /RTC1 /D_DEBUG")
    set(_msvc_release_flags "/O2 /Ob2 /DNDEBUG")
    foreach(_config DEBUG debug)
      set(CMAKE_CXX_FLAGS_${_config} "${_msvc_debug_flags}" CACHE STRING "" FORCE)
      set(CMAKE_C_FLAGS_${_config} "${_msvc_debug_flags}" CACHE STRING "" FORCE)
    endforeach()
    foreach(_config CHECKED PROFILE RELEASE checked profile release)
      set(CMAKE_CXX_FLAGS_${_config} "${_msvc_release_flags}" CACHE STRING "" FORCE)
      set(CMAKE_C_FLAGS_${_config} "${_msvc_release_flags}" CACHE STRING "" FORCE)
    endforeach()
  else()
    set(CMAKE_CXX_FLAGS_DEBUG "-g -D_DEBUG" CACHE STRING "" FORCE)
    set(CMAKE_C_FLAGS_DEBUG "-g -D_DEBUG" CACHE STRING "" FORCE)
    foreach(_config CHECKED PROFILE RELEASE)
      set(CMAKE_CXX_FLAGS_${_config} "-O3 -DNDEBUG" CACHE STRING "" FORCE)
      set(CMAKE_C_FLAGS_${_config} "-O3 -DNDEBUG" CACHE STRING "" FORCE)
    endforeach()
  endif()
endif()

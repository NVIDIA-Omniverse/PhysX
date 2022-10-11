if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 2.5)
   message(FATAL_ERROR "CMake >= 2.6.0 required")
endif()
cmake_policy(PUSH)
cmake_policy(VERSION 2.6)

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Compute the installation prefix relative to this file.
get_filename_component(_IMPORT_PREFIX "${CMAKE_CURRENT_LIST_FILE}" PATH)
if(_IMPORT_PREFIX STREQUAL "/")
  set(_IMPORT_PREFIX "")
endif()

SET(TARGA_INCLUDE_DIR "${_IMPORT_PREFIX}/include")

SET(TARGA_SRC_DIR "${_IMPORT_PREFIX}/src")

MESSAGE("Targa at ${_IMPORT_PREFIX}")

# This file does not depend on other imported targets which have
# been exported from the same project but in a separate export set.

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
cmake_policy(POP)
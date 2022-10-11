if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 2.5)
   message(FATAL_ERROR "CMake >= 2.6.0 required")
endif()
cmake_policy(PUSH)
cmake_policy(VERSION 2.6)


# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Protect against multiple inclusion, which would fail when already imported targets are added once more.
set(_targetsDefined)
set(_targetsNotDefined)
set(_expectedTargets)
foreach(_expectedTarget GL GLU GLUT Xmu Xi)
  list(APPEND _expectedTargets ${_expectedTarget})
  if(NOT TARGET ${_expectedTarget})
    list(APPEND _targetsNotDefined ${_expectedTarget})
  endif()
  if(TARGET ${_expectedTarget})
    list(APPEND _targetsDefined ${_expectedTarget})
  endif()
endforeach()
if("${_targetsDefined}" STREQUAL "${_expectedTargets}")
  unset(_targetsDefined)
  unset(_targetsNotDefined)
  unset(_expectedTargets)
  set(CMAKE_IMPORT_FILE_VERSION)
  cmake_policy(POP)
  return()
endif()
if(NOT "${_targetsDefined}" STREQUAL "")
  message(FATAL_ERROR "Some (but not all) targets in this export set were already defined.\nTargets Defined: ${_targetsDefined}\nTargets not yet defined: ${_targetsNotDefined}\n")
endif()
unset(_targetsDefined)
unset(_targetsNotDefined)
unset(_expectedTargets)

# Compute the installation prefix relative to this file.
get_filename_component(_IMPORT_PREFIX "${CMAKE_CURRENT_LIST_FILE}" PATH)
if(_IMPORT_PREFIX STREQUAL "/")
  set(_IMPORT_PREFIX "")
endif()

# Create imported target Cg
add_library(GL SHARED IMPORTED)
add_library(GLU SHARED IMPORTED)

set_target_properties(GL PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${_IMPORT_PREFIX}/include;${_IMPORT_PREFIX}/include/GL"
)

# Import target "GL" for configuration "debug"
set_property(TARGET GL APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(GL PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
  IMPORTED_LOCATION   "${_IMPORT_PREFIX}/lib64/libGL.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS GL )

set_target_properties(GLU PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${_IMPORT_PREFIX}/include;${_IMPORT_PREFIX}/include/GL"
)

# Import target "GLU" for configuration "debug"
set_property(TARGET GLU APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(GLU PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
  IMPORTED_LOCATION   "${_IMPORT_PREFIX}/lib64/libGLU.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS GLU )


# These 3 go together
add_library(GLUT SHARED IMPORTED)
add_library(Xmu SHARED IMPORTED)
add_library(Xi SHARED IMPORTED)

# glut
set_property(TARGET GLUT PROPERTY IMPORTED_LINK_INTERFACE_LIBRARIES Xmu Xi)

set_target_properties(GLUT PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${_IMPORT_PREFIX}/include;${_IMPORT_PREFIX}/include/GL"
)

# Import target "GLUT" for configuration "debug"
set_property(TARGET GLUT APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(GLUT PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
  IMPORTED_LOCATION   "${_IMPORT_PREFIX}/lib64/libglut.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS GLUT )

# Xmu
set_target_properties(Xmu PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${_IMPORT_PREFIX}/include;${_IMPORT_PREFIX}/include/GL"
)

# Import target "Xmu" for configuration "debug"
set_property(TARGET Xmu APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(Xmu PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
  IMPORTED_LOCATION   "${_IMPORT_PREFIX}/lib64/libXmu.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS Xmu )

#Xi

set_target_properties(Xi PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${_IMPORT_PREFIX}/include;${_IMPORT_PREFIX}/include/GL"
)

# Import target "Xi" for configuration "debug"
set_property(TARGET Xi APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(Xi PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
  IMPORTED_LOCATION   "${_IMPORT_PREFIX}/lib64/libXi.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS Xi )


# Cleanup temporary variables.
set(_IMPORT_PREFIX)

# Loop over all imported files and verify that they actually exist
foreach(target ${_IMPORT_CHECK_TARGETS} )
  foreach(file ${_IMPORT_CHECK_FILES_FOR_${target}} )
    if(NOT EXISTS "${file}" )
      message(FATAL_ERROR "The imported target \"${target}\" references the file
   \"${file}\"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   \"${CMAKE_CURRENT_LIST_FILE}\"
but not all the files it references.
")
    endif()
  endforeach()
  unset(_IMPORT_CHECK_FILES_FOR_${target})
endforeach()
unset(_IMPORT_CHECK_TARGETS)

# This file does not depend on other imported targets which have
# been exported from the same project but in a separate export set.

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
cmake_policy(POP)
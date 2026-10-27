## SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
## SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.21)

# Only call project() if this is the top-level CMakeLists.txt
# When used via FetchContent, the parent already called project()
# Calling project() again would reset CMAKE_CXX_FLAGS_* variables
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(PhysX C CXX)
endif()

CMAKE_POLICY(SET CMP0057 NEW) # Enable IN_LIST

# Check if this is a public release build
IF(NOT DEFINED PUBLIC_RELEASE)
    SET(PUBLIC_RELEASE 0) # Default to private release
ENDIF()
IF(PUBLIC_RELEASE)
    MESSAGE("Building for PUBLIC RELEASE")
ENDIF()

OPTION(PX_SCALAR_MATH "Disable SIMD math" OFF)
OPTION(PX_GENERATE_STATIC_LIBRARIES "Generate static libraries" OFF)
OPTION(PX_EXPORT_LOWLEVEL_PDB "Export low level pdb's" OFF)

IF(NOT DEFINED PHYSX_ROOT_DIR)

	STRING(REPLACE "\\" "/" BRD_TEMP $ENV{PHYSX_ROOT_DIR})

	# This env variable is set by GenerateProjects.bat, and is no longer available when CMake rebuilds, so this stores it in the cache
	SET(PHYSX_ROOT_DIR ${BRD_TEMP} CACHE INTERNAL "Root of the PhysX source tree")

ENDIF()

MESSAGE("PHYSX ROOT ${PHYSX_ROOT_DIR}")

IF(NOT EXISTS ${PHYSX_ROOT_DIR})
	MESSAGE(FATAL_ERROR "PHYSX_ROOT_DIR environment variable wasn't set or was invalid.")
ENDIF()

MESSAGE("PhysX Build Platform: " ${TARGET_BUILD_PLATFORM})
MESSAGE("Using CXX Compiler: " ${CMAKE_CXX_COMPILER})

# NvidiaBuildOptions.cmake sets up WINCRT_DEBUG/WINCRT_NDEBUG variables,
# CMAKE_*_POSTFIX values, and platform-specific compiler flags.
# PhysXConfigurationTypes.cmake is NOT included here because it would
# FORCE-overwrite the flags NvidiaBuildOptions just set. It is only used
# by the FetchContent entry point (physx/CMakeLists.txt) where
# NvidiaBuildOptions is not included.
INCLUDE(NvidiaBuildOptions)

IF(CMAKE_CONFIGURATION_TYPES)
	SET(CMAKE_CONFIGURATION_TYPES debug checked profile release)
	SET(CMAKE_CONFIGURATION_TYPES "${CMAKE_CONFIGURATION_TYPES}" CACHE STRING
		"Reset config to what we need"
		FORCE)

	SET(CMAKE_SHARED_LINKER_FLAGS_CHECKED "")
	SET(CMAKE_SHARED_LINKER_FLAGS_PROFILE "")

	# Build PDBs for all configurations
	SET(CMAKE_SHARED_LINKER_FLAGS "/DEBUG")

ENDIF()

# Prevent failure due to command line limitations
IF(USE_RESPONSE_FILES)
	SET(CMAKE_C_USE_RESPONSE_FILE_FOR_OBJECTS 1)
	SET(CMAKE_C_USE_RESPONSE_FILE_FOR_INCLUDES 1)
	SET(CMAKE_C_USE_RESPONSE_FILE_FOR_LIBRARIES 1)
	SET(CMAKE_CXX_USE_RESPONSE_FILE_FOR_OBJECTS 1)
	SET(CMAKE_CXX_USE_RESPONSE_FILE_FOR_INCLUDES 1)
	SET(CMAKE_CXX_USE_RESPONSE_FILE_FOR_LIBRARIES 1)
ENDIF()

IF($ENV{PHYSX_AUTOBUILD})
  IF($ENV{PHYSX_AUTOBUILD} STREQUAL "1")
  	SET(PHYSX_AUTOBUILD "PHYSX_AUTOBUILD")
  ENDIF()
ENDIF()

SET(PROJECT_CMAKE_FILES_DIR source/compiler/cmake)
SET(PLATFORM_CMAKELISTS ${PHYSX_ROOT_DIR}/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/CMakeLists.txt)

IF(NOT EXISTS ${PLATFORM_CMAKELISTS})
	MESSAGE(FATAL_ERROR "Unable to find platform CMakeLists.txt for ${TARGET_BUILD_PLATFORM} at ${PLATFORM_CMAKELISTS}")
ENDIF()

SET(CMAKE_POSITION_INDEPENDENT_CODE ON)

SET(SOURCE_DISTRO_FILE_LIST "")

SET(HEADER_GUARD_NAME "CONFIG")
SET(HEADER_CONTENT "")
FILE(READ ${PHYSX_ROOT_DIR}/buildtools/templates/boilerplate_apache.txt BOILERPLATE_CONTENT)

# Include the platform specific CMakeLists
INCLUDE(${PHYSX_ROOT_DIR}/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/CMakeLists.txt)

# generate PxConfig.h header that will contain PhysX configuration defines like PX_PHYSX_STATIC_LIB
CONFIGURE_FILE(${PHYSX_ROOT_DIR}/buildtools/templates/PxIncludeTemplate.h ${PHYSX_ROOT_DIR}/include/PxConfig.h)

IF(PX_GENERATE_SOURCE_DISTRO)
	FOREACH(FILE_NAME ${SOURCE_DISTRO_FILE_LIST})
		FILE(APPEND "${CMAKE_CURRENT_BINARY_DIR}/source_distro_list.txt" "${FILE_NAME}\n")
	ENDFOREACH()
ENDIF()

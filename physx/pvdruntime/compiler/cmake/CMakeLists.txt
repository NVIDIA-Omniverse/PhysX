## SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
## SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.21)

project(PVDRuntime C CXX)

set(PVDRuntimeBuilt 1 CACHE INTERNAL "PVDRuntimeBuilt")

# This is required to be defined by external callers!
IF(NOT DEFINED PHYSX_ROOT_DIR)
	MESSAGE(FATAL_ERROR "PHYSX_ROOT_DIR variable wasn't set.")
ENDIF()

IF(NOT EXISTS ${PHYSX_ROOT_DIR})
	MESSAGE(FATAL_ERROR "PHYSX_ROOT_DIR variable was invalid.")
ENDIF()

INCLUDE(NvidiaBuildOptions)

IF(CMAKE_CONFIGURATION_TYPES)
	SET(CMAKE_CONFIGURATION_TYPES debug checked profile release)
	SET(CMAKE_CONFIGURATION_TYPES "${CMAKE_CONFIGURATION_TYPES}" CACHE STRING
		"Reset config to what we need" 
		FORCE)
	
	# Need to define these at least once.
	SET(CMAKE_EXE_LINKER_FLAGS_CHECKED "")
	SET(CMAKE_EXE_LINKER_FLAGS_PROFILE "")

	# Build PDBs for all configurations
	SET(CMAKE_EXE_LINKER_FLAGS "/DEBUG")
	
ENDIF()

SET(PROJECT_CMAKE_FILES_DIR compiler/cmake)
SET(PLATFORM_CMAKELISTS ${PHYSX_ROOT_DIR}/pvdruntime/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/CMakeLists.txt)

IF(NOT EXISTS ${PLATFORM_CMAKELISTS})
	MESSAGE(FATAL_ERROR "Unable to find platform CMakeLists.txt for ${TARGET_BUILD_PLATFORM} at ${PLATFORM_CMAKELISTS}")
ENDIF()

# Include the platform specific CMakeLists
INCLUDE(${PHYSX_ROOT_DIR}/pvdruntime/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/CMakeLists.txt)

# Set folder Server to all Server projects
SET_PROPERTY(TARGET PVDRuntime PROPERTY FOLDER "PVDRuntime")

IF(PX_GENERATE_SOURCE_DISTRO)
	FOREACH(FILE_NAME ${SOURCE_DISTRO_FILE_LIST})
		FILE(APPEND "${CMAKE_CURRENT_BINARY_DIR}/source_distro_list.txt" "${FILE_NAME}\n")
	ENDFOREACH()
ENDIF()


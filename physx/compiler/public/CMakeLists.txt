## SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
## SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.21)

IF(NOT DEFINED PHYSX_CMAKE_MODULES_PATH OR PHYSX_CMAKE_MODULES_PATH STREQUAL "")
	IF(DEFINED ENV{PM_cmake_PATH} AND NOT "$ENV{PM_cmake_PATH}" STREQUAL "")
		LIST(APPEND PHYSX_CMAKE_MODULES_PATH "$ENV{PM_cmake_PATH}/share/cmake-3.25/Modules")
	ENDIF()
	LIST(APPEND PHYSX_CMAKE_MODULES_PATH ${PHYSX_ROOT_DIR}/source/compiler/cmake/modules  CACHE INTERNAL "Path to PhysX CMake Modules")
ENDIF()

LIST(APPEND CMAKE_MODULE_PATH ${PHYSX_CMAKE_MODULES_PATH})

project(PhysXSDK C CXX)

OPTION(PX_BUILDSNIPPETS "Generate the snippets" OFF)
OPTION(PX_BUILDPVDRUNTIME "Generate the OmniPVD project" OFF)
OPTION(PX_CMAKE_SUPPRESS_REGENERATION "Disable zero_check projects" OFF)

SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)

IF(NOT CMAKE_CROSSCOMPILING AND (TARGET_BUILD_PLATFORM STREQUAL "windows" OR TARGET_BUILD_PLATFORM STREQUAL "linux"))
	OPTION(PX_GENERATE_GPU_PROJECTS "Generate the GPU projects, if possible." OFF)
	OPTION(PX_GENERATE_GPU_PROJECTS_ONLY "Generate ONLY the GPU projects, if possible." OFF)

	IF(PX_GENERATE_GPU_PROJECTS OR PX_GENERATE_GPU_PROJECTS_ONLY)
		# Will enable building CUDA files ( .cu )
		ENABLE_LANGUAGE(CUDA)
		# The FIND_PACKAGE(CUDAToolkit) makes it easier if you have CUDA language enabled to link to
		# CTK libraries by providing nice targets like CUDA::cudart
		FIND_PACKAGE(CUDAToolkit REQUIRED)

		# We need to set CMAKE_CUDA_ARCHITECTURES with a default value
		# see policy CMP0104
		IF(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
			SET(CMAKE_CUDA_ARCHITECTURES 75)
		ENDIF()
	ENDIF()
ENDIF()

# This is required to be defined by external callers!
IF(NOT DEFINED PHYSX_ROOT_DIR)
	MESSAGE(FATAL_ERROR "PHYSX_ROOT_DIR variable wasn't set.")
ENDIF()

IF(NOT EXISTS ${PHYSX_ROOT_DIR})
	MESSAGE(FATAL_ERROR "PHYSX_ROOT_DIR variable was invalid.")
ENDIF()

SET(PUBLIC_RELEASE 1)

# Enable folder properties
SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

# Disable zero_check projects. The default for Switch is ON.
IF(PX_CMAKE_SUPPRESS_REGENERATION)
	SET(CMAKE_SUPPRESS_REGENERATION true)
ENDIF()

# Add PhysX SDK Source code to solution
ADD_SUBDIRECTORY(${PHYSX_ROOT_DIR}/source/compiler/cmake ${CMAKE_CURRENT_BINARY_DIR}/sdk_source_bin)
MESSAGE("Added PhysX")

IF(PX_GENERATE_GPU_PROJECTS OR PX_GENERATE_GPU_PROJECTS_ONLY)
	# Add PhysX SDK Gpu Source code to solution
	ADD_SUBDIRECTORY(${PHYSX_ROOT_DIR}/source/compiler/cmakegpu ${CMAKE_CURRENT_BINARY_DIR}/sdk_gpu_source_bin)
	MESSAGE("Added PhysX GPU")
ENDIF()


IF(PX_BUILDPVDRUNTIME)
	IF(TARGET_BUILD_PLATFORM STREQUAL "windows" OR TARGET_BUILD_PLATFORM STREQUAL "linux")
		ADD_SUBDIRECTORY(${PHYSX_ROOT_DIR}/pvdruntime/compiler/cmake ${CMAKE_CURRENT_BINARY_DIR}/pvdruntime_bin)
		MESSAGE("Added PVDRuntime")
		# PhysX statically links the OmniPVD runtime; the PhysX SDK build declares the
		# dependency here so PVDRuntime.cmake stays free of any PhysX knowledge.
		IF(TARGET PhysX)
			TARGET_LINK_LIBRARIES(PhysX PRIVATE PVDRuntime)
		ENDIF()
		IF(NOT DEFINED PX_ENABLE_INSTALL)
			INSTALL(TARGETS PVDRuntime
				EXPORT PhysXSDK
				ARCHIVE DESTINATION "$<$<CONFIG:debug>:${PX_ROOT_LIB_DIR}/debug>$<$<CONFIG:release>:${PX_ROOT_LIB_DIR}/release>$<$<CONFIG:checked>:${PX_ROOT_LIB_DIR}/checked>$<$<CONFIG:profile>:${PX_ROOT_LIB_DIR}/profile>"
				INCLUDES DESTINATION pvdruntime/include
			)
		ENDIF()
	ENDIF()
ENDIF()

IF(PX_BUILDSNIPPETS)
	# Add Snippets projects into the solution
	ADD_SUBDIRECTORY(${PHYSX_ROOT_DIR}/snippets/compiler/cmake ${CMAKE_CURRENT_BINARY_DIR}/sdk_snippets_bin)

	MESSAGE("Added Snippets")
ENDIF()


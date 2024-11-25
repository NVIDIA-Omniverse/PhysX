## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions
## are met:
##  * Redistributions of source code must retain the above copyright
##    notice, this list of conditions and the following disclaimer.
##  * Redistributions in binary form must reproduce the above copyright
##    notice, this list of conditions and the following disclaimer in the
##    documentation and/or other materials provided with the distribution.
##  * Neither the name of NVIDIA CORPORATION nor the names of its
##    contributors may be used to endorse or promote products derived
##    from this software without specific prior written permission.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
## EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
## IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
## PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
## CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
## EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
## PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
## PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
## OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
## (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
## OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
##
## Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.

cmake_minimum_required(VERSION 3.16)

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

IF(PX_CMAKE_SUPPRESS_REGENERATION)
	SET(CMAKE_SUPPRESS_REGENERATION true)
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

IF(PX_BUILDPVDRUNTIME)
	IF(TARGET_BUILD_PLATFORM STREQUAL "windows" OR TARGET_BUILD_PLATFORM STREQUAL "linux")
		ADD_SUBDIRECTORY(${PHYSX_ROOT_DIR}/pvdruntime/compiler/cmake ${CMAKE_CURRENT_BINARY_DIR}/pvdruntime_bin)
		MESSAGE("Added PVDRuntime")
	ENDIF()
ENDIF()

IF(PX_BUILDSNIPPETS)
	# Add Snippets projects into the solution
	ADD_SUBDIRECTORY(${PHYSX_ROOT_DIR}/snippets/compiler/cmake ${CMAKE_CURRENT_BINARY_DIR}/sdk_snippets_bin)

	MESSAGE("Added Snippets")
ENDIF()


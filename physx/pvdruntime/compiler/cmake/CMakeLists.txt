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
## Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.

cmake_minimum_required(VERSION 3.7)

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
	SET(CMAKE_SHARED_LINKER_FLAGS_CHECKED "")
	SET(CMAKE_SHARED_LINKER_FLAGS_PROFILE "")

	# Build PDBs for all configurations
	SET(CMAKE_EXE_LINKER_FLAGS "/DEBUG")
	SET(CMAKE_SHARED_LINKER_FLAGS "/DEBUG")
	
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

INSTALL(
		TARGETS PVDRuntime
		EXPORT PhysXSDK
		DESTINATION $<$<CONFIG:debug>:${PX_ROOT_LIB_DIR}/debug>$<$<CONFIG:release>:${PX_ROOT_LIB_DIR}/release>$<$<CONFIG:checked>:${PX_ROOT_LIB_DIR}/checked>$<$<CONFIG:profile>:${PX_ROOT_LIB_DIR}/profile>	
		)
      
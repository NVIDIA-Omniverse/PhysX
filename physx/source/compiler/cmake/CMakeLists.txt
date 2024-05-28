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

cmake_minimum_required(VERSION 3.7)

project(PhysX C CXX)

CMAKE_POLICY(SET CMP0057 NEW) # Enable IN_LIST

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
FILE(READ ${PHYSX_ROOT_DIR}/buildtools/templates/boilerplate_bsd.txt BOILERPLATE_CONTENT)

# Include the platform specific CMakeLists
INCLUDE(${PHYSX_ROOT_DIR}/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/CMakeLists.txt)

# generate PxConfig.h header that will contain PhysX configuration defines like PX_PHYSX_STATIC_LIB
CONFIGURE_FILE(${PHYSX_ROOT_DIR}/buildtools/templates/PxIncludeTemplate.h ${PHYSX_ROOT_DIR}/include/PxConfig.h)

IF(PX_GENERATE_SOURCE_DISTRO)
	FOREACH(FILE_NAME ${SOURCE_DISTRO_FILE_LIST})
		FILE(APPEND "${CMAKE_CURRENT_BINARY_DIR}/source_distro_list.txt" "${FILE_NAME}\n")
	ENDFOREACH()
ENDIF()
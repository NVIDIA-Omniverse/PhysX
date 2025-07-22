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
## Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.

cmake_minimum_required(VERSION 3.16)

# PROJECT(CUDA) will enable building CUDA files ( .cu )
PROJECT(PhysX C CXX CUDA)

IF(DEFINED ENV{PM_winsdk_PATH} AND NOT "$ENV{PM_winsdk_PATH}" STREQUAL "" AND DEFINED ENV{PM_msvc_PATH} AND NOT "$ENV{PM_msvc_PATH}" STREQUAL "")
	SET(CMAKE_VS_SDK_INCLUDE_DIRECTORIES "$ENV{PM_winsdk_PATH}/include/ucrt;$ENV{PM_winsdk_PATH}/include/um;$ENV{PM_winsdk_PATH}/include/shared")
	SET(CMAKE_VS_SDK_LIBRARY_DIRECTORIES "$ENV{PM_winsdk_PATH}/lib/ucrt/x64;$ENV{PM_winsdk_PATH}/lib/um/x64;$ENV{VCToolsInstallDir}/lib/x64;$ENV{VCToolsInstallDir}/atlmfc/lib/x64")
ENDIF()

OPTION(PX_GENERATE_GPU_STATIC_LIBRARIES "Generate PhysXGPU static libraries" OFF)
OPTION(PX_GENERATE_GPU_REDUCED_ARCHITECTURES "Generate only a reduced number of GPU architectures for faster compilation" OFF)

CMAKE_POLICY(SET CMP0057 NEW) # Enable IN_LIST

# This is required to be defined by external callers!
IF(NOT DEFINED PHYSX_ROOT_DIR)
	MESSAGE(FATAL_ERROR "PHYSX_ROOT_DIR variable wasn't set.")
ENDIF()

IF(NOT EXISTS ${PHYSX_ROOT_DIR})
	MESSAGE(FATAL_ERROR "PHYSX_ROOT_DIR variable was invalid.")
ENDIF()

INCLUDE(NvidiaBuildOptions)
INCLUDE(SetCudaArch)

SET(CMAKE_POSITION_INDEPENDENT_CODE ON)

IF(CMAKE_CONFIGURATION_TYPES)
	SET(CMAKE_CONFIGURATION_TYPES debug checked profile release)
	SET(CMAKE_CONFIGURATION_TYPES "${CMAKE_CONFIGURATION_TYPES}" CACHE STRING
		"Reset config to what we need"
		FORCE)

	# Need to define these at least once.
	SET(CMAKE_SHARED_LINKER_FLAGS_CHECKED "/DEBUG")
	SET(CMAKE_SHARED_LINKER_FLAGS_PROFILE "/DEBUG")
	SET(CMAKE_EXE_LINKER_FLAGS_PROFILE "/DEBUG")
	SET(CMAKE_EXE_LINKER_FLAGS_CHECKED "/DEBUG")
ENDIF()

SET(PROJECT_CMAKE_FILES_DIR source/compiler/cmakegpu)
SET(PLATFORM_CMAKELISTS ${PHYSX_ROOT_DIR}/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/CMakeLists.txt)

IF(NOT EXISTS ${PLATFORM_CMAKELISTS})
	MESSAGE(FATAL_ERROR "Unable to find platform CMakeLists.txt for ${TARGET_BUILD_PLATFORM} at ${PLATFORM_CMAKELISTS}")
ENDIF()

SET(SOURCE_DISTRO_FILE_LIST "")

# CUDA Flags that are common to all platforms:
# Note: We need to compile to sass for all architectures we want to support.
# However, to enable forward compatibility with new architectures,
# we need to compile ptx for the latest arch supported by the cuda toolkit.
# This will allow the jit compiler in the driver to compile ptx to sass of the newer arch.

# No need to set --generate-code=arch=compute_75,code=[compute_75,sm_75]
# since that's the default set in: compiler\internal\CMakeLists.txt (using CMAKE_CUDA_ARCHITECTURES)
# see policy CMP0104
IF(PX_GENERATE_GPU_REDUCED_ARCHITECTURES)
	GENERATE_ARCH_CODE_LIST(SASS "80,86,89,90,100,120" PTX "120")
ELSE()
	# Volta is the minimum compute arch required because NGC supports V100
	GENERATE_ARCH_CODE_LIST(SASS "70,80,86,89,90,100,120" PTX "120")
ENDIF()

# Force response files off because clangd does not parse them
set(CMAKE_CUDA_USE_RESPONSE_FILE_FOR_INCLUDES 0)
set(CMAKE_CUDA_USE_RESPONSE_FILE_FOR_LIBRARIES 0)
set(CMAKE_CUDA_USE_RESPONSE_FILE_FOR_OBJECTS 0)

# Cuda setup that is the same for all platforms and sub-projects
SET(PHYSX_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Werror=all-warnings -use_fast_math -ftz=true -prec-div=false -prec-sqrt=false -t0 -D_CONSOLE" CACHE INTERNAL "PhysX CUDA")
# Include the platform specific CMakeLists (The other CUDA flags that are specific to each platform are defined there)
INCLUDE(${PHYSX_ROOT_DIR}/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/CMakeLists.txt)

IF(PX_GENERATE_SOURCE_DISTRO)
	FOREACH(FILE_NAME ${SOURCE_DISTRO_FILE_LIST})
		FILE(APPEND "${CMAKE_CURRENT_BINARY_DIR}/source_distro_list.txt" "${FILE_NAME}\n")
	ENDFOREACH()
ENDIF()

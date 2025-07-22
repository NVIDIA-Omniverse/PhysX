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

#
# Build PhysXGpu common
#

SET(PHYSX_SOURCE_DIR ${PHYSX_ROOT_DIR}/source)

# Conditionally set the device source directory based on PUBLIC_RELEASE
IF(PUBLIC_RELEASE)
    SET(DEVICE_INCLUDE_DIR ${PHYSX_SOURCE_DIR}/physx/src/opensource/cudamanager/include)
ELSE()
    SET(DEVICE_INCLUDE_DIR ${PHYSX_SOURCE_DIR}/physx/src/internal/cudamanager/include)
ENDIF()

# Include here after the directories are defined so that the platform specific file can use the variables.
include(${PHYSX_ROOT_DIR}/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/PhysXGpu.cmake)

# gpu
SET(PHYXGPU_HEADERS
	${PHYSX_SOURCE_DIR}/physxgpu/include/PxPhysXGpu.h
	${PHYSX_SOURCE_DIR}/physxgpu/src/PxgPhysXGpu.h
)
SOURCE_GROUP("include" FILES ${PHYXGPU_HEADERS})

SET(PHYXGPU_SOURCE
	${PHYSX_SOURCE_DIR}/physxgpu/src/PxgPhysXGpu.cpp
)
SOURCE_GROUP("src" FILES ${PHYXGPU_SOURCE})

ADD_LIBRARY(PhysXGpu ${PHYSXGPU_LIBTYPE}
	${PHYSXGPU_PLATFORM_SOURCES}
	${PHYXGPU_HEADERS}
	${PHYXGPU_SOURCE}
	$<TARGET_OBJECTS:PhysXArticulationGpu>
	$<TARGET_OBJECTS:PhysXBroadphaseGpu>
	$<TARGET_OBJECTS:PhysXCommonGpu>
	$<TARGET_OBJECTS:PhysXCudaContextManager>
	$<TARGET_OBJECTS:PhysXNarrowphaseGpu>
	$<TARGET_OBJECTS:PhysXSimulationControllerGpu>
	$<TARGET_OBJECTS:PhysXSolverGpu>
	$<TARGET_OBJECTS:PhysXGpuDependencies>
)

TARGET_INCLUDE_DIRECTORIES(PhysXGpu
	PRIVATE ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
	PRIVATE ${PHYSX_ROOT_DIR}/include
	PRIVATE ${PHYSX_SOURCE_DIR}/common/include
	PRIVATE ${PHYSX_SOURCE_DIR}/common/src
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/include
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/contact
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/pcm
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/mesh
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevel/api/include
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevel/api/include/windows
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevel/common/include/pipeline
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevel/common/include/utils
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevelaabb/include
	PRIVATE ${PHYSX_SOURCE_DIR}/lowleveldynamics/include
	PRIVATE ${PHYSX_SOURCE_DIR}/lowleveldynamics/shared
	PRIVATE ${PHYSX_SOURCE_DIR}/lowleveldynamics/src
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevel/software/include
	PRIVATE ${PHYSX_SOURCE_DIR}/gpusolver/include
	PRIVATE ${PHYSX_SOURCE_DIR}/gpubroadphase/include
	PRIVATE ${PHYSX_SOURCE_DIR}/gpunarrowphase/include
	PRIVATE ${PHYSX_SOURCE_DIR}/gpucommon/include
	PRIVATE ${PHYSX_SOURCE_DIR}/gpucommon/src/CUDA
	PRIVATE ${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include
	PRIVATE ${PHYSX_SOURCE_DIR}/gpuarticulation/include
	PRIVATE ${PHYSX_SOURCE_DIR}/cudamanager/include
	PRIVATE ${PHYSX_SOURCE_DIR}/physxgpu/include
)

# Add CUDA manager include directory
IF(PUBLIC_RELEASE)
    TARGET_INCLUDE_DIRECTORIES(PhysXGpu PRIVATE ${PHYSX_SOURCE_DIR}/physx/src/opensource/cudamanager/include)
ELSE()
    TARGET_INCLUDE_DIRECTORIES(PhysXGpu PRIVATE ${PHYSX_SOURCE_DIR}/physx/src/internal/cudamanager/include)
ENDIF()

SET_TARGET_PROPERTIES(PhysXGpu PROPERTIES
	OUTPUT_NAME PhysXGpu
)

IF(PHYSXGPU_LIBTYPE STREQUAL "STATIC")
	SET_TARGET_PROPERTIES(PhysXGpu PROPERTIES
		ARCHIVE_OUTPUT_NAME_DEBUG "PhysXGpu_static"
		ARCHIVE_OUTPUT_NAME_CHECKED "PhysXGpu_static"
		ARCHIVE_OUTPUT_NAME_PROFILE "PhysXGpu_static"
		ARCHIVE_OUTPUT_NAME_RELEASE "PhysXGpu_static"
	)
ENDIF()

IF(PHYSXGPU_COMPILE_PDB_NAME_DEBUG)
	SET_TARGET_PROPERTIES(PhysXGpu PROPERTIES
		COMPILE_PDB_NAME_DEBUG ${PHYSXGPU_COMPILE_PDB_NAME_DEBUG}
		COMPILE_PDB_NAME_CHECKED ${PHYSXGPU_COMPILE_PDB_NAME_CHECKED}
		COMPILE_PDB_NAME_PROFILE ${PHYSXGPU_COMPILE_PDB_NAME_PROFILE}
		COMPILE_PDB_NAME_RELEASE ${PHYSXGPU_COMPILE_PDB_NAME_RELEASE}
	)
ENDIF()

TARGET_COMPILE_DEFINITIONS(PhysXGpu
	PRIVATE ${PHYSXGPU_COMPILE_DEFS}
)

SET_TARGET_PROPERTIES(PhysXGpu PROPERTIES
	LINK_FLAGS ${PHYSXGPU_LINK_FLAGS}
	LINK_FLAGS_DEBUG ${PHYSXGPU_LINK_FLAGS_DEBUG}
	LINK_FLAGS_CHECKED ${PHYSXGPU_LINK_FLAGS_CHECKED}
	LINK_FLAGS_PROFILE ${PHYSXGPU_LINK_FLAGS_PROFILE}
	LINK_FLAGS_RELEASE ${PHYSXGPU_LINK_FLAGS_RELEASE}
)

# Add linked libraries
TARGET_LINK_LIBRARIES(PhysXGpu
	PRIVATE ${PHYSXGPU_PRIVATE_PLATFORM_LINKED_LIBS}
	PUBLIC ${PHYSXGPU_PLATFORM_LINKED_LIBS}
)

IF(PX_GENERATE_SOURCE_DISTRO)
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYXGPU_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYXGPU_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSXGPU_PLATFORM_SOURCES})
ENDIF()

# enable -fPIC so we can link static libs with the editor
SET_TARGET_PROPERTIES(PhysXGpu PROPERTIES
	POSITION_INDEPENDENT_CODE TRUE
)


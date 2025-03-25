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
# Build PhysXBroadphaseGpu common
#

SET(PHYSX_SOURCE_DIR ${PHYSX_ROOT_DIR}/source)
SET(BROAD_PHASE_SOURCE_DIR ${PHYSX_SOURCE_DIR}/gpubroadphase/src)

# Include here after the directories are defined so that the platform specific file can use the variables.
include(${PHYSX_ROOT_DIR}/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/PhysXBroadphaseGpu.cmake)

# setup grouping
# broadphase
SET(PHYXGPU_BROADPHASE_HEADERS
	${PHYSX_SOURCE_DIR}/gpubroadphase/include/PxgBroadPhaseCommonDefines.h
	${PHYSX_SOURCE_DIR}/gpubroadphase/include/PxgBroadPhaseDesc.h
	${PHYSX_SOURCE_DIR}/gpubroadphase/include/PxgBroadPhaseKernelIndices.h
	${PHYSX_SOURCE_DIR}/gpubroadphase/include/PxgBroadPhasePairReport.h
	${PHYSX_SOURCE_DIR}/gpubroadphase/include/PxgBroadPhase.h
	${PHYSX_SOURCE_DIR}/gpubroadphase/include/PxgCudaBroadPhaseSap.h
	${PHYSX_SOURCE_DIR}/gpubroadphase/include/PxgIntegerAABB.h
	${PHYSX_SOURCE_DIR}/gpubroadphase/include/PxgSapBox1D.h
	${PHYSX_SOURCE_DIR}/gpubroadphase/include/PxgAggregate.h
	${PHYSX_SOURCE_DIR}/gpubroadphase/include/PxgAABBManager.h
	${PHYSX_SOURCE_DIR}/gpubroadphase/include/PxgAggregateDesc.h
)
SOURCE_GROUP("broadphase include" FILES ${PHYXGPU_BROADPHASE_HEADERS})

SET(PHYXGPU_BROADPHASE_CUDA_KERNELS
	${BROAD_PHASE_SOURCE_DIR}/CUDA/broadphase.cu
	${BROAD_PHASE_SOURCE_DIR}/CUDA/aggregate.cu
)
SOURCE_GROUP("broadphase kernels/CUDA" FILES ${PHYXGPU_BROADPHASE_CUDA_KERNELS})

SET(PHYXGPU_BROADPHASE_SOURCE
	${BROAD_PHASE_SOURCE_DIR}/PxgCudaBroadPhaseSap.cpp
	${BROAD_PHASE_SOURCE_DIR}/PxgBroadPhase.cpp
	${BROAD_PHASE_SOURCE_DIR}/PxgAABBManager.cpp
)
SOURCE_GROUP("broadphase src" FILES ${PHYXGPU_BROADPHASE_SOURCE})

ADD_LIBRARY(PhysXBroadphaseGpu ${PHYSXBROADPHASEGPU_LIBTYPE}
	# broadphase
	${PHYXGPU_BROADPHASE_HEADERS}
	${PHYXGPU_BROADPHASE_CUDA_KERNELS}
	${PHYXGPU_BROADPHASE_SOURCE}
)

TARGET_INCLUDE_DIRECTORIES(PhysXBroadphaseGpu
	PRIVATE ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
	PRIVATE ${PHYSX_ROOT_DIR}/include
	PRIVATE ${PHYSX_SOURCE_DIR}/common/include
	PRIVATE ${PHYSX_SOURCE_DIR}/common/src
	PRIVATE ${PHYSX_SOURCE_DIR}/physxgpu/include
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
	PRIVATE ${PHYSX_SOURCE_DIR}/cudamanager/include
)

SET_TARGET_PROPERTIES(PhysXBroadphaseGpu PROPERTIES
	OUTPUT_NAME PhysXBroadphaseGpu
)


IF(PHYSXBROADPHASEGPU_LIBTYPE STREQUAL "STATIC")
	SET_TARGET_PROPERTIES(PhysXBroadphaseGpu PROPERTIES
		ARCHIVE_OUTPUT_NAME_DEBUG "PhysXBroadphaseGpu_static"
		ARCHIVE_OUTPUT_NAME_CHECKED "PhysXBroadphaseGpu_static"
		ARCHIVE_OUTPUT_NAME_PROFILE "PhysXBroadphaseGpu_static"
		ARCHIVE_OUTPUT_NAME_RELEASE "PhysXBroadphaseGpu_static"
	)
ENDIF()

IF(PHYSXBROADPHASEGPU_COMPILE_PDB_NAME_DEBUG)
	SET_TARGET_PROPERTIES(PhysXBroadphaseGpu PROPERTIES
		COMPILE_PDB_NAME_DEBUG "${PHYSXBROADPHASEGPU_COMPILE_PDB_NAME_DEBUG}"
		COMPILE_PDB_NAME_CHECKED "${PHYSXBROADPHASEGPU_COMPILE_PDB_NAME_CHECKED}"
		COMPILE_PDB_NAME_PROFILE "${PHYSXBROADPHASEGPU_COMPILE_PDB_NAME_PROFILE}"
		COMPILE_PDB_NAME_RELEASE "${PHYSXBROADPHASEGPU_COMPILE_PDB_NAME_RELEASE}"
	)
ENDIF()

TARGET_COMPILE_DEFINITIONS(PhysXBroadphaseGpu
	# Common to all configurations
	PRIVATE ${PHYSXBROADPHASEGPU_COMPILE_DEFS}
)

# Since we are setting the C++ standard explicitly for Linux
# we need to do this for CUDA as well.
IF(TARGET_BUILD_PLATFORM STREQUAL "linux")
	TARGET_COMPILE_FEATURES(PhysXBroadphaseGpu PRIVATE cuda_std_11)
ENDIF()

TARGET_COMPILE_OPTIONS(PhysXBroadphaseGpu PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:${ARCH_CODE_LIST}>)

IF(PX_GENERATE_SOURCE_DISTRO)
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYXGPU_BROADPHASE_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYXGPU_BROADPHASE_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYXGPU_BROADPHASE_CUDA_KERNELS})
ENDIF()

# enable -fPIC so we can link static libs with the editor
SET_TARGET_PROPERTIES(PhysXBroadphaseGpu PROPERTIES
	POSITION_INDEPENDENT_CODE TRUE
)


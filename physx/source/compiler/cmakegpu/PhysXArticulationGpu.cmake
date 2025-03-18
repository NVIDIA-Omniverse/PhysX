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
# Build PhysXArticulationGpu common
#

SET(PHYSX_SOURCE_DIR ${PHYSX_ROOT_DIR}/source)
SET(GPUARTICULATION_SOURCE_DIR ${PHYSX_SOURCE_DIR}/gpuarticulation/src)

# Include here after the directories are defined so that the platform specific file can use the variables.
include(${PHYSX_ROOT_DIR}/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/PhysXArticulationGpu.cmake)


# setup grouping
# articulation
SET(PHYXGPU_ARTICULATION_HEADERS
	${PHYSX_SOURCE_DIR}/gpuarticulation/include/PxgArticulationCore.h
	${PHYSX_SOURCE_DIR}/gpuarticulation/include/PxgArticulationCoreDesc.h
	${PHYSX_SOURCE_DIR}/gpuarticulation/include/PxgArticulationCoreKernelIndices.h
)
SOURCE_GROUP("articulation include" FILES ${PHYXGPU_ARTICULATION_HEADERS})

SET(PHYXGPU_ARTICULATION_CUDA_KERNELS
	${GPUARTICULATION_SOURCE_DIR}/CUDA/articulationDirectGpuApi.cu
	${GPUARTICULATION_SOURCE_DIR}/CUDA/forwardDynamic2.cu
	${GPUARTICULATION_SOURCE_DIR}/CUDA/internalConstraints2.cu
	${GPUARTICULATION_SOURCE_DIR}/CUDA/inverseDynamic.cu
)
SOURCE_GROUP("articulation kernels/CUDA" FILES ${PHYXGPU_ARTICULATION_CUDA_KERNELS})

SET(PHYXGPU_ARTICULATION_CUDA_INCLUDE
	${GPUARTICULATION_SOURCE_DIR}/CUDA/articulationDynamic.cuh
	${GPUARTICULATION_SOURCE_DIR}/CUDA/articulationImpulseResponse.cuh
)
SOURCE_GROUP("articulation kernels cuda include" FILES ${PHYXGPU_ARTICULATION_CUDA_INCLUDE})

SET(PHYXGPU_ARTICULATION_SOURCE
	${GPUARTICULATION_SOURCE_DIR}/PxgArticulationCore.cpp
)
SOURCE_GROUP("articulation src" FILES ${PHYXGPU_ARTICULATION_SOURCE})

ADD_LIBRARY(PhysXArticulationGpu ${PHYSXARTICULATIONGPU_LIBTYPE}
	# ARTICULATION
	${PHYXGPU_ARTICULATION_HEADERS}
	${PHYXGPU_ARTICULATION_CUDA_KERNELS}
	${PHYXGPU_ARTICULATION_CUDA_INCLUDE}
	${PHYXGPU_ARTICULATION_SOURCE}
)

TARGET_INCLUDE_DIRECTORIES(PhysXArticulationGpu
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
	PRIVATE ${PHYSX_SOURCE_DIR}/gpusolver/src/CUDA
	PRIVATE ${PHYSX_SOURCE_DIR}/gpubroadphase/include
	PRIVATE ${PHYSX_SOURCE_DIR}/gpunarrowphase/include
	PRIVATE ${PHYSX_SOURCE_DIR}/gpucommon/include
	PRIVATE ${PHYSX_SOURCE_DIR}/gpucommon/src/CUDA
	PRIVATE ${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include
	PRIVATE ${PHYSX_SOURCE_DIR}/gpusimulationcontroller/src/CUDA
	PRIVATE ${PHYSX_SOURCE_DIR}/gpuarticulation/include
	PRIVATE ${PHYSX_SOURCE_DIR}/cudamanager/include
)

SET_TARGET_PROPERTIES(PhysXArticulationGpu PROPERTIES
	OUTPUT_NAME PhysXArticulationGpu
)


IF(PHYSXARTICULATIONGPU_LIBTYPE STREQUAL "STATIC")
	SET_TARGET_PROPERTIES(PhysXArticulationGpu PROPERTIES
		ARCHIVE_OUTPUT_NAME_DEBUG "PhysXArticulationGpu_static"
		ARCHIVE_OUTPUT_NAME_CHECKED "PhysXArticulationGpu_static"
		ARCHIVE_OUTPUT_NAME_PROFILE "PhysXArticulationGpu_static"
		ARCHIVE_OUTPUT_NAME_RELEASE "PhysXArticulationGpu_static"
	)
ENDIF()

IF(PHYSXARTICULATIONGPU_COMPILE_PDB_NAME_DEBUG)
	SET_TARGET_PROPERTIES(PhysXArticulationGpu PROPERTIES
		COMPILE_PDB_NAME_DEBUG "${PHYSXARTICULATIONGPU_COMPILE_PDB_NAME_DEBUG}"
		COMPILE_PDB_NAME_CHECKED "${PHYSXARTICULATIONGPU_COMPILE_PDB_NAME_CHECKED}"
		COMPILE_PDB_NAME_PROFILE "${PHYSXARTICULATIONGPU_COMPILE_PDB_NAME_PROFILE}"
		COMPILE_PDB_NAME_RELEASE "${PHYSXARTICULATIONGPU_COMPILE_PDB_NAME_RELEASE}"
	)
ENDIF()


TARGET_COMPILE_DEFINITIONS(PhysXArticulationGpu
	PRIVATE ${PHYSXARTICULATIONGPU_COMPILE_DEFS}
)

# Since we are setting the C++ standard explicitly for Linux
# we need to do this for CUDA as well.
IF(TARGET_BUILD_PLATFORM STREQUAL "linux")
	TARGET_COMPILE_FEATURES(PhysXArticulationGpu PRIVATE cuda_std_11)
ENDIF()

TARGET_COMPILE_OPTIONS(PhysXArticulationGpu PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:${ARCH_CODE_LIST}>)

IF(PX_GENERATE_SOURCE_DISTRO)
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYXGPU_ARTICULATION_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYXGPU_ARTICULATION_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYXGPU_ARTICULATION_CUDA_INCLUDE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYXGPU_ARTICULATION_CUDA_KERNELS})
ENDIF()

# enable -fPIC so we can link static libs with the editor
SET_TARGET_PROPERTIES(PhysXArticulationGpu PROPERTIES
	POSITION_INDEPENDENT_CODE TRUE
)

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
SET(GPU_COMMON_SOURCE_DIR ${PHYSX_SOURCE_DIR}/gpucommon/src)

# Include here after the directories are defined so that the platform specific file can use the variables.
include(${PHYSX_ROOT_DIR}/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/PhysXCommonGpu.cmake)


# setup grouping
# common
SET(PHYXGPU_COMMON_HEADERS
	${PHYSX_SOURCE_DIR}/gpucommon/include/AlignedMat33.h
	${PHYSX_SOURCE_DIR}/gpucommon/include/AlignedQuat.h
	${PHYSX_SOURCE_DIR}/gpucommon/include/AlignedTransform.h
	${PHYSX_SOURCE_DIR}/gpucommon/include/cutil_math.h
	${PHYSX_SOURCE_DIR}/gpucommon/include/mathsExtensions.h
	${PHYSX_SOURCE_DIR}/gpucommon/include/PxgCommon.h
	${PHYSX_SOURCE_DIR}/gpucommon/include/PxgCommonDefines.h
	${PHYSX_SOURCE_DIR}/gpucommon/include/PxgContactsDebug.h
	${PHYSX_SOURCE_DIR}/gpucommon/include/PxgCopyManager.h
	${PHYSX_SOURCE_DIR}/gpucommon/include/PxgCudaBuffer.h
	${PHYSX_SOURCE_DIR}/gpucommon/include/PxgCudaHelpers.h
	${PHYSX_SOURCE_DIR}/gpucommon/include/PxgCudaMemoryAllocator.h
	${PHYSX_SOURCE_DIR}/gpucommon/include/PxgCudaPagedFirstFitHoleAllocator.h
	${PHYSX_SOURCE_DIR}/gpucommon/include/PxgCudaPagedLinearAllocator.h
	${PHYSX_SOURCE_DIR}/gpucommon/include/PxgCudaUtils.h
	${PHYSX_SOURCE_DIR}/gpucommon/include/PxgHeapMemAllocator.h
	${PHYSX_SOURCE_DIR}/gpucommon/include/PxgKernelIndices.h
	${PHYSX_SOURCE_DIR}/gpucommon/include/PxgIntrinsics.h
	${PHYSX_SOURCE_DIR}/gpucommon/include/PxgKernelWrangler.h
	${PHYSX_SOURCE_DIR}/gpucommon/include/PxgKernelNames.h
	${PHYSX_SOURCE_DIR}/gpucommon/include/PxgMemCopyDispatcher.h
	${PHYSX_SOURCE_DIR}/gpucommon/include/PxgMemoryManager.h
	${PHYSX_SOURCE_DIR}/gpucommon/include/PxgRadixSortDesc.h
	${PHYSX_SOURCE_DIR}/gpucommon/include/PxgRadixSortKernelIndices.h
	${PHYSX_SOURCE_DIR}/gpucommon/include/PxSpatialMatrix.h
    ${PHYSX_SOURCE_DIR}/gpucommon/include/PxgDevicePointer.h
)
SOURCE_GROUP("common include" FILES ${PHYXGPU_COMMON_HEADERS})

SET(PHYXGPU_COMMON_CUDA_KERNELS
	${GPU_COMMON_SOURCE_DIR}/CUDA/MemCopyBalanced.cu
	${GPU_COMMON_SOURCE_DIR}/CUDA/radixSortImpl.cu
    ${GPU_COMMON_SOURCE_DIR}/CUDA/utility.cu
)
SOURCE_GROUP("common kernels/CUDA" FILES ${PHYXGPU_COMMON_CUDA_KERNELS})

SET(PHYXGPU_COMMON_CUDA_INCLUDE
	${GPU_COMMON_SOURCE_DIR}/CUDA/shuffle.cuh
	${GPU_COMMON_SOURCE_DIR}/CUDA/atomic.cuh
	${GPU_COMMON_SOURCE_DIR}/CUDA/reduction.cuh
	${GPU_COMMON_SOURCE_DIR}/CUDA/contactReduction.cuh
	${GPU_COMMON_SOURCE_DIR}/CUDA/updateCacheAndBound.cuh
	${GPU_COMMON_SOURCE_DIR}/CUDA/copy.cuh
	${GPU_COMMON_SOURCE_DIR}/CUDA/utils.cuh
	${GPU_COMMON_SOURCE_DIR}/CUDA/vector.cuh
	${GPU_COMMON_SOURCE_DIR}/CUDA/SparseRemove.cuh
	${GPU_COMMON_SOURCE_DIR}/CUDA/RadixSort.cuh
	${GPU_COMMON_SOURCE_DIR}/CUDA/MemoryAllocator.cuh
	${GPU_COMMON_SOURCE_DIR}/CUDA/gridCal.cuh
	${GPU_COMMON_SOURCE_DIR}/CUDA/sbMidphaseScratch.cuh
	${GPU_COMMON_SOURCE_DIR}/CUDA/femMidphaseScratch.cuh
	${GPU_COMMON_SOURCE_DIR}/CUDA/solverResidual.cuh
)
SOURCE_GROUP("common kernels cuda include" FILES ${PHYXGPU_COMMON_CUDA_INCLUDE})

SET(PHYXGPU_COMMON_SOURCE
	${PHYSX_GPU_PLATFORM_SOURCE}
	${GPU_COMMON_SOURCE_DIR}/PxgCopyManager.cpp
	${GPU_COMMON_SOURCE_DIR}/PxgCudaBuffer.cpp
	${GPU_COMMON_SOURCE_DIR}/PxgCudaMemoryAllocator.cpp
	${GPU_COMMON_SOURCE_DIR}/PxgHeapMemoryAllocator.cpp
	${GPU_COMMON_SOURCE_DIR}/PxgKernelWrangler.cpp
	${GPU_COMMON_SOURCE_DIR}/PxgMemCopyDispatcher.cpp
	${GPU_COMMON_SOURCE_DIR}/PxgMemoryManager.cpp
	${GPU_COMMON_SOURCE_DIR}/PxgCommon.cpp
)
SOURCE_GROUP("common src" FILES ${PHYXGPU_COMMON_SOURCE})

ADD_LIBRARY(PhysXCommonGpu ${PHYSXCOMMONGPU_LIBTYPE}
	# GPU common
	${PHYXGPU_COMMON_HEADERS}
	${PHYXGPU_COMMON_CUDA_KERNELS}
	${PHYXGPU_COMMON_CUDA_INCLUDE}
	${PHYXGPU_COMMON_SOURCE}
)

TARGET_INCLUDE_DIRECTORIES(PhysXCommonGpu
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
	PRIVATE ${PHYSX_SOURCE_DIR}/gpunarrowphase/src/CUDA
	PRIVATE ${PHYSX_SOURCE_DIR}/gpucommon/include
	PRIVATE ${PHYSX_SOURCE_DIR}/gpucommon/src/CUDA
	PRIVATE ${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include
	PRIVATE ${PHYSX_SOURCE_DIR}/cudamanager/include
)

SET_TARGET_PROPERTIES(PhysXCommonGpu PROPERTIES
	OUTPUT_NAME PhysXCommonGpu
)


IF(PHYSXCOMMONGPU_LIBTYPE STREQUAL "STATIC")
	SET_TARGET_PROPERTIES(PhysXCommonGpu PROPERTIES
		ARCHIVE_OUTPUT_NAME_DEBUG "PhysXCommonGpu_static"
		ARCHIVE_OUTPUT_NAME_CHECKED "PhysXCommonGpu_static"
		ARCHIVE_OUTPUT_NAME_PROFILE "PhysXCommonGpu_static"
		ARCHIVE_OUTPUT_NAME_RELEASE "PhysXCommonGpu_static"
	)
ENDIF()

IF(PHYSXCOMMONGPU_COMPILE_PDB_NAME_DEBUG)
	SET_TARGET_PROPERTIES(PhysXCommonGpu PROPERTIES
		COMPILE_PDB_NAME_DEBUG "${PHYSXCOMMONGPU_COMPILE_PDB_NAME_DEBUG}"
		COMPILE_PDB_NAME_CHECKED "${PHYSXCOMMONGPU_COMPILE_PDB_NAME_CHECKED}"
		COMPILE_PDB_NAME_PROFILE "${PHYSXCOMMONGPU_COMPILE_PDB_NAME_PROFILE}"
		COMPILE_PDB_NAME_RELEASE "${PHYSXCOMMONGPU_COMPILE_PDB_NAME_RELEASE}"
	)
ENDIF()

TARGET_COMPILE_DEFINITIONS(PhysXCommonGpu
	PRIVATE ${PHYSXCOMMONGPU_COMPILE_DEFS}
)

# Since we are setting the C++ standard explicitly for Linux
# we need to do this for CUDA as well.
IF(TARGET_BUILD_PLATFORM STREQUAL "linux")
	TARGET_COMPILE_FEATURES(PhysXCommonGpu PRIVATE cuda_std_11)
ENDIF()

TARGET_COMPILE_OPTIONS(PhysXCommonGpu PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:${ARCH_CODE_LIST}>)

IF(PX_GENERATE_SOURCE_DISTRO)
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYXGPU_COMMON_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYXGPU_COMMON_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYXGPU_COMMON_CUDA_INCLUDE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYXGPU_COMMON_CUDA_KERNELS})
ENDIF()

# enable -fPIC so we can link static libs with the editor
SET_TARGET_PROPERTIES(PhysXCommonGpu PROPERTIES
	POSITION_INDEPENDENT_CODE TRUE
)


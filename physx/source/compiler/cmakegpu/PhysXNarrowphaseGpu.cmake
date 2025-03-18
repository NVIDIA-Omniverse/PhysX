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
SET(NARROW_PHASE_SOURCE_DIR ${PHYSX_SOURCE_DIR}/gpunarrowphase/src)

# Include here after the directories are defined so that the platform specific file can use the variables.
include(${PHYSX_ROOT_DIR}/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/PhysXNarrowphaseGpu.cmake)


# setup grouping
# narrowphase
SET(PHYXGPU_NARROWPHASE_HEADERS
	${PHYSX_SOURCE_DIR}/gpunarrowphase/include/convexFormat.h
	${PHYSX_SOURCE_DIR}/gpunarrowphase/include/convexNpCommon.h
	${PHYSX_SOURCE_DIR}/gpunarrowphase/include/cudaNpCommon.h
	${PHYSX_SOURCE_DIR}/gpunarrowphase/include/PxgContactManager.h
	${PHYSX_SOURCE_DIR}/gpunarrowphase/include/PxgConvexConvexShape.h
	${PHYSX_SOURCE_DIR}/gpunarrowphase/include/PxgGeometryManager.h
	${PHYSX_SOURCE_DIR}/gpunarrowphase/include/PxgNarrowphaseCore.h
	${PHYSX_SOURCE_DIR}/gpunarrowphase/include/PxgNphaseImplementationContext.h
	${PHYSX_SOURCE_DIR}/gpunarrowphase/include/PxgNpKernelIndices.h
	${PHYSX_SOURCE_DIR}/gpunarrowphase/include/PxgPersistentContactManifold.h
	${PHYSX_SOURCE_DIR}/gpunarrowphase/include/PxgShapeManager.h
	${PHYSX_SOURCE_DIR}/gpunarrowphase/include/schlockShared.h
	${PHYSX_SOURCE_DIR}/gpunarrowphase/include/tri32Data.h
	${PHYSX_SOURCE_DIR}/gpunarrowphase/include/typeHelpers.h
	${PHYSX_SOURCE_DIR}/gpunarrowphase/include/PxgNarrowphase.h
)
SOURCE_GROUP("narrowphase include" FILES ${PHYXGPU_NARROWPHASE_HEADERS})

SET(PHYXGPU_NARROWPHASE_CUDA_KERNELS
	${NARROW_PHASE_SOURCE_DIR}/CUDA/convexHFMidphase.cu
	${NARROW_PHASE_SOURCE_DIR}/CUDA/convexHeightfield.cu
	${NARROW_PHASE_SOURCE_DIR}/CUDA/convexMesh.cu
	${NARROW_PHASE_SOURCE_DIR}/CUDA/convexMeshCorrelate.cu
	${NARROW_PHASE_SOURCE_DIR}/CUDA/convexMeshMidphase.cu
	${NARROW_PHASE_SOURCE_DIR}/CUDA/convexMeshPostProcess.cu
	${NARROW_PHASE_SOURCE_DIR}/CUDA/convexMeshOutput.cu
	${NARROW_PHASE_SOURCE_DIR}/CUDA/cudaGJKEPA.cu
	${NARROW_PHASE_SOURCE_DIR}/CUDA/convexCoreCollision.cu
	${NARROW_PHASE_SOURCE_DIR}/CUDA/cudaSphere.cu
    ${NARROW_PHASE_SOURCE_DIR}/CUDA/cudaBox.cu
	${NARROW_PHASE_SOURCE_DIR}/CUDA/compressOutputContacts.cu
	${NARROW_PHASE_SOURCE_DIR}/CUDA/pairManagement.cu
	${NARROW_PHASE_SOURCE_DIR}/CUDA/cudaParticleSystem.cu
	${NARROW_PHASE_SOURCE_DIR}/CUDA/particleSystemMeshMidphase.cu
	${NARROW_PHASE_SOURCE_DIR}/CUDA/particleSystemHFMidPhaseCG.cu
	${NARROW_PHASE_SOURCE_DIR}/CUDA/softbodyMidPhase.cu
	${NARROW_PHASE_SOURCE_DIR}/CUDA/softbodyPrimitives.cu
	${NARROW_PHASE_SOURCE_DIR}/CUDA/softbodySoftbodyMidPhase.cu
	${NARROW_PHASE_SOURCE_DIR}/CUDA/softbodyHFMidPhase.cu
	${NARROW_PHASE_SOURCE_DIR}/CUDA/femClothMidPhase.cu
	${NARROW_PHASE_SOURCE_DIR}/CUDA/femClothPrimitives.cu
	${NARROW_PHASE_SOURCE_DIR}/CUDA/femClothHFMidPhase.cu
	${NARROW_PHASE_SOURCE_DIR}/CUDA/femClothClothMidPhase.cu
	${NARROW_PHASE_SOURCE_DIR}/CUDA/trimeshCollision.cu
)
SOURCE_GROUP("narrowphase kernels/CUDA" FILES ${PHYXGPU_NARROWPHASE_CUDA_KERNELS})

SET(PHYXGPU_NARROWPHASE_CUDA_INCLUDE
	${NARROW_PHASE_SOURCE_DIR}/CUDA/triangle.cuh
	${NARROW_PHASE_SOURCE_DIR}/CUDA/triangletriangle.cuh
	${NARROW_PHASE_SOURCE_DIR}/CUDA/distanceSegmentSegment.cuh
	${NARROW_PHASE_SOURCE_DIR}/CUDA/convexTriangle.cuh
	${NARROW_PHASE_SOURCE_DIR}/CUDA/sphereTriangle.cuh
	${NARROW_PHASE_SOURCE_DIR}/CUDA/capsuleTriangle.cuh
	${NARROW_PHASE_SOURCE_DIR}/CUDA/dataReadWriteHelper.cuh
	${NARROW_PHASE_SOURCE_DIR}/CUDA/epa.cuh
	${NARROW_PHASE_SOURCE_DIR}/CUDA/gjk.cuh
	${NARROW_PHASE_SOURCE_DIR}/CUDA/heightfieldUtil.cuh
	${NARROW_PHASE_SOURCE_DIR}/CUDA/manifold.cuh
	${NARROW_PHASE_SOURCE_DIR}/CUDA/materialCombiner.cuh
	${NARROW_PHASE_SOURCE_DIR}/CUDA/midphaseAllocate.cuh
	${NARROW_PHASE_SOURCE_DIR}/CUDA/nputils.cuh
	${NARROW_PHASE_SOURCE_DIR}/CUDA/warpHelpers.cuh
	${NARROW_PHASE_SOURCE_DIR}/CUDA/deformableElementFilter.cuh
	${NARROW_PHASE_SOURCE_DIR}/CUDA/contactPatchUtils.cuh
	${NARROW_PHASE_SOURCE_DIR}/CUDA/bv32Traversal.cuh
	${NARROW_PHASE_SOURCE_DIR}/CUDA/sdfCollision.cuh
	${NARROW_PHASE_SOURCE_DIR}/CUDA/triangleMesh.cuh
	${NARROW_PHASE_SOURCE_DIR}/CUDA/deformableCollision.cuh
	${NARROW_PHASE_SOURCE_DIR}/CUDA/sphereCollision.cuh
	${NARROW_PHASE_SOURCE_DIR}/CUDA/particleCollision.cuh
)
SOURCE_GROUP("narrowphase kernels cuda include" FILES ${PHYXGPU_NARROWPHASE_CUDA_INCLUDE})

SET(PHYXGPU_NARROWPHASE_SOURCE
	${NARROW_PHASE_SOURCE_DIR}/PxgGeometryManager.cpp
	${NARROW_PHASE_SOURCE_DIR}/PxgNarrowphaseCore.cpp
	${NARROW_PHASE_SOURCE_DIR}/PxgNarrowphase.cpp
	${NARROW_PHASE_SOURCE_DIR}/PxgNphaseImplementationContext.cpp
	${NARROW_PHASE_SOURCE_DIR}/PxgShapeManager.cpp
)
SOURCE_GROUP("narrowphase src" FILES ${PHYXGPU_NARROWPHASE_SOURCE})

ADD_LIBRARY(PhysXNarrowphaseGpu ${PHYSXNARROWPHASEGPU_LIBTYPE}
	# narrowphase
	${PHYXGPU_NARROWPHASE_CUDA_INCLUDE}
	${PHYXGPU_NARROWPHASE_CUDA_KERNELS}
	${PHYXGPU_NARROWPHASE_SOURCE}
	${PHYXGPU_NARROWPHASE_HEADERS}
)
TARGET_INCLUDE_DIRECTORIES(PhysXNarrowphaseGpu
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
	PRIVATE ${PHYSX_SOURCE_DIR}/gpusimulationcontroller/src/CUDA
	PRIVATE ${PHYSX_SOURCE_DIR}/gpuarticulation/include
	PRIVATE ${PHYSX_SOURCE_DIR}/cudamanager/include
)

SET_TARGET_PROPERTIES(PhysXNarrowphaseGpu PROPERTIES
	OUTPUT_NAME PhysXNarrowphaseGpu
)


IF(PHYSXNARROWPHASEGPU_LIBTYPE STREQUAL "STATIC")
	SET_TARGET_PROPERTIES(PhysXNarrowphaseGpu PROPERTIES
		ARCHIVE_OUTPUT_NAME_DEBUG "PhysXNarrowphaseGpu_static"
		ARCHIVE_OUTPUT_NAME_CHECKED "PhysXNarrowphaseGpu_static"
		ARCHIVE_OUTPUT_NAME_PROFILE "PhysXNarrowphaseGpu_static"
		ARCHIVE_OUTPUT_NAME_RELEASE "PhysXNarrowphaseGpu_static"
	)
ENDIF()

IF(PHYSXNPGPU_COMPILE_PDB_NAME_DEBUG)
	SET_TARGET_PROPERTIES(PhysXNarrowphaseGpu PROPERTIES
		COMPILE_PDB_NAME_DEBUG "${PHYSXNPGPU_COMPILE_PDB_NAME_DEBUG}"
		COMPILE_PDB_NAME_CHECKED "${PHYSXNPGPU_COMPILE_PDB_NAME_CHECKED}"
		COMPILE_PDB_NAME_PROFILE "${PHYSXNPGPU_COMPILE_PDB_NAME_PROFILE}"
		COMPILE_PDB_NAME_RELEASE "${PHYSXNPGPU_COMPILE_PDB_NAME_RELEASE}"
	)
ENDIF()


TARGET_COMPILE_DEFINITIONS(PhysXNarrowphaseGpu
	PRIVATE ${PHYSXNARROWPHASEGPU_COMPILE_DEFS}
)

# Since we are setting the C++ standard explicitly for Linux
# we need to do this for CUDA as well.
IF(TARGET_BUILD_PLATFORM STREQUAL "linux")
	TARGET_COMPILE_FEATURES(PhysXNarrowphaseGpu PRIVATE cuda_std_11)
ENDIF()

TARGET_COMPILE_OPTIONS(PhysXNarrowphaseGpu PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:${ARCH_CODE_LIST}>)

IF(PX_GENERATE_SOURCE_DISTRO)
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYXGPU_NARROWPHASE_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYXGPU_NARROWPHASE_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYXGPU_NARROWPHASE_CUDA_INCLUDE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYXGPU_NARROWPHASE_CUDA_KERNELS})
ENDIF()

# enable -fPIC so we can link static libs with the editor
SET_TARGET_PROPERTIES(PhysXNarrowphaseGpu PROPERTIES
	POSITION_INDEPENDENT_CODE TRUE
)

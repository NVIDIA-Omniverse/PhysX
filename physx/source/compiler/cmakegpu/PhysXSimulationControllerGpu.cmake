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
# Build PhysXSimulationControllerGpu common
#

SET(PHYSX_SOURCE_DIR ${PHYSX_ROOT_DIR}/source)
SET(GPUSIMCTRL_SOURCE_DIR ${PHYSX_SOURCE_DIR}/gpusimulationcontroller/src)

# Include here after the directories are defined so that the platform specific file can use the variables.
include(${PHYSX_ROOT_DIR}/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/PhysXSimulationControllerGpu.cmake)


# setup grouping
# simulation controller
SET(PHYXGPU_SIMCONTROLLER_HEADERS
	${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgBodySim.h
	${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgBodySimManager.h
	${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgArticulation.h
	${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgArticulationLink.h
	${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgArticulationTendon.h
	${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgSoftBody.h
	${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgFEMCloth.h
	${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgParticleSystem.h
	${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgSimulationCore.h
	${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgJointManager.h
	${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgShapeSim.h
	${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgShapeSimManager.h
	${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgSimulationController.h
	${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgSimulationCoreDesc.h
	${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgSimulationCoreKernelIndices.h
	${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgParticleSystemCoreKernelIndices.h
	${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgParticleSystemCore.h
	${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgPBDParticleSystemCore.h
	${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgSoftBodyCore.h
	${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgSoftBodyCoreKernelIndices.h
	${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgFEMClothCore.h
	${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgFEMClothCoreKernelIndices.h
	${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgNonRigidCoreCommon.h
	${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgFEMCore.h
    ${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgKernelLauncher.h
    ${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgRadixSortCore.h
    ${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgEssentialCore.h
    ${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgAlgorithms.h
    ${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgAlgorithmsData.h
    ${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgParticleNeighborhoodProvider.h
    ${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgAnisotropy.h
    ${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgAnisotropyData.h
    ${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgSparseGridStandalone.h
    ${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgSmoothing.h
    ${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgParticleNeighborhoodProvider.h
    ${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgIsosurfaceExtraction.h
    ${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgDenseGridData.h
    ${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgDenseGridDataStandalone.h
    ${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgIsosurfaceData.h
    ${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgInterpolation.h
    ${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgSparseGridDataStandalone.h
    ${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgArrayConverter.h
	${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgSDFBuilder.h
	${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgBVH.h
    ${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgDeformableSkinning.h
    ${PHYSX_SOURCE_DIR}/gpusimulationcontroller/include/PxgConstraintIdMap.h
)
SOURCE_GROUP("simulation controller include" FILES ${PHYXGPU_SIMCONTROLLER_HEADERS})

SET(PHYXGPU_SIMCONTROLLER_CUDA_KERNELS
	${GPUSIMCTRL_SOURCE_DIR}/CUDA/updateTransformAndBoundArray.cu
	${GPUSIMCTRL_SOURCE_DIR}/CUDA/updateBodiesAndShapes.cu
	${GPUSIMCTRL_SOURCE_DIR}/CUDA/particlesystem.cu
	${GPUSIMCTRL_SOURCE_DIR}/CUDA/softBody.cu
	${GPUSIMCTRL_SOURCE_DIR}/CUDA/softBodyGM.cu
	${GPUSIMCTRL_SOURCE_DIR}/CUDA/FEMClothConstraintPrep.cu
	${GPUSIMCTRL_SOURCE_DIR}/CUDA/FEMCloth.cu
	${GPUSIMCTRL_SOURCE_DIR}/CUDA/FEMClothExternalSolve.cu
	${GPUSIMCTRL_SOURCE_DIR}/CUDA/rigidDeltaAccum.cu
	${GPUSIMCTRL_SOURCE_DIR}/CUDA/diffuseParticles.cu
	${GPUSIMCTRL_SOURCE_DIR}/CUDA/isosurfaceExtraction.cu
	${GPUSIMCTRL_SOURCE_DIR}/CUDA/anisotropy.cu
	${GPUSIMCTRL_SOURCE_DIR}/CUDA/algorithms.cu
	${GPUSIMCTRL_SOURCE_DIR}/CUDA/sparseGridStandalone.cu
	${GPUSIMCTRL_SOURCE_DIR}/CUDA/SDFConstruction.cu
)
SOURCE_GROUP("simulation controller kernels/CUDA" FILES ${PHYXGPU_SIMCONTROLLER_CUDA_KERNELS})

SET(PHYXGPU_SIMCONTROLLER_CUDA_INCLUDE
	${GPUSIMCTRL_SOURCE_DIR}/CUDA/attachments.cuh
	${GPUSIMCTRL_SOURCE_DIR}/CUDA/deformableUtils.cuh
	${GPUSIMCTRL_SOURCE_DIR}/CUDA/particleSystem.cuh
	${GPUSIMCTRL_SOURCE_DIR}/CUDA/softBody.cuh
	${GPUSIMCTRL_SOURCE_DIR}/CUDA/FEMClothUtil.cuh
	${GPUSIMCTRL_SOURCE_DIR}/CUDA/matrixDecomposition.cuh
	${GPUSIMCTRL_SOURCE_DIR}/CUDA/marchingCubesTables.cuh
	${GPUSIMCTRL_SOURCE_DIR}/CUDA/denseGridStandalone.cuh
	${GPUSIMCTRL_SOURCE_DIR}/CUDA/sparseGridStandalone.cuh
	${GPUSIMCTRL_SOURCE_DIR}/CUDA/bvh.cuh
    ${GPUSIMCTRL_SOURCE_DIR}/CUDA/FEMClothUtil.cuh
)
SOURCE_GROUP("simulation controller cuda include" FILES ${PHYXGPU_SIMCONTROLLER_CUDA_INCLUDE})

SET(PHYXGPU_SIMCONTROLLER_SOURCE
	${GPUSIMCTRL_SOURCE_DIR}/PxgBodySimManager.cpp
	${GPUSIMCTRL_SOURCE_DIR}/PxgSimulationCore.cpp
	${GPUSIMCTRL_SOURCE_DIR}/PxgJointManager.cpp
	${GPUSIMCTRL_SOURCE_DIR}/PxgShapeSimManager.cpp
	${GPUSIMCTRL_SOURCE_DIR}/PxgSimulationController.cpp
	${GPUSIMCTRL_SOURCE_DIR}/PxgParticleSystemCore.cpp
	${GPUSIMCTRL_SOURCE_DIR}/PxgPBDParticleSystemCore.cpp
	${GPUSIMCTRL_SOURCE_DIR}/PxgSoftBodyCore.cpp
	${GPUSIMCTRL_SOURCE_DIR}/PxgSoftBody.cpp
	${GPUSIMCTRL_SOURCE_DIR}/PxgFEMClothCore.cpp
	${GPUSIMCTRL_SOURCE_DIR}/PxgFEMCloth.cpp
	${GPUSIMCTRL_SOURCE_DIR}/PxgNonRigidCoreCommon.cpp
	${GPUSIMCTRL_SOURCE_DIR}/PxgFEMCore.cpp
    ${GPUSIMCTRL_SOURCE_DIR}/PxgRadixSortCore.cpp
    ${GPUSIMCTRL_SOURCE_DIR}/PxgAlgorithms.cpp
    ${GPUSIMCTRL_SOURCE_DIR}/PxgAnisotropy.cpp
    ${GPUSIMCTRL_SOURCE_DIR}/PxgParticleNeighborhood.cpp
    ${GPUSIMCTRL_SOURCE_DIR}/PxgSmoothing.cpp
    ${GPUSIMCTRL_SOURCE_DIR}/PxgSparseGridStandalone.cpp
    ${GPUSIMCTRL_SOURCE_DIR}/PxgIsosurfaceExtraction.cpp
    ${GPUSIMCTRL_SOURCE_DIR}/PxgArrayConverter.cpp
	${GPUSIMCTRL_SOURCE_DIR}/PxgSDFBuilder.cpp
    ${GPUSIMCTRL_SOURCE_DIR}/PxgDeformableSkinning.cpp
)
SOURCE_GROUP("simulation controller src" FILES ${PHYXGPU_SIMCONTROLLER_SOURCE})

ADD_LIBRARY(PhysXSimulationControllerGpu ${PHYSXSIMULATIONCONTROLLERGPU_LIBTYPE}
	# Simulation controller
	${PHYXGPU_SIMCONTROLLER_HEADERS}
	${PHYXGPU_SIMCONTROLLER_CUDA_KERNELS}
	${PHYXGPU_SIMCONTROLLER_CUDA_INCLUDE}
	${PHYXGPU_SIMCONTROLLER_SOURCE}
)

TARGET_INCLUDE_DIRECTORIES(PhysXSimulationControllerGpu
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
	PRIVATE ${PHYSX_SOURCE_DIR}/gpuarticulation/include
)

SET_TARGET_PROPERTIES(PhysXSimulationControllerGpu PROPERTIES
	OUTPUT_NAME PhysXSimulationControllerGpu
)


IF(PHYSXSIMULATIONCONTROLLERGPU_LIBTYPE STREQUAL "STATIC")
	SET_TARGET_PROPERTIES(PhysXSimulationControllerGpu PROPERTIES
		ARCHIVE_OUTPUT_NAME_DEBUG "PhysXSimulationControllerGpu_static"
		ARCHIVE_OUTPUT_NAME_CHECKED "PhysXSimulationControllerGpu_static"
		ARCHIVE_OUTPUT_NAME_PROFILE "PhysXSimulationControllerGpu_static"
		ARCHIVE_OUTPUT_NAME_RELEASE "PhysXSimulationControllerGpu_static"
	)
ENDIF()

IF(PHYSXSCGPU_COMPILE_PDB_NAME_DEBUG)
	SET_TARGET_PROPERTIES(PhysXSimulationControllerGpu PROPERTIES
		COMPILE_PDB_NAME_DEBUG "${PHYSXSCGPU_COMPILE_PDB_NAME_DEBUG}"
		COMPILE_PDB_NAME_CHECKED "${PHYSXSCGPU_COMPILE_PDB_NAME_CHECKED}"
		COMPILE_PDB_NAME_PROFILE "${PHYSXSCGPU_COMPILE_PDB_NAME_PROFILE}"
		COMPILE_PDB_NAME_RELEASE "${PHYSXSCGPU_COMPILE_PDB_NAME_RELEASE}"
	)
ENDIF()

TARGET_COMPILE_DEFINITIONS(PhysXSimulationControllerGpu
	PRIVATE ${PHYSXSIMULATIONCONTROLLERGPU_COMPILE_DEFS}
)

# Since we are setting the C++ standard explicitly for Linux
# we need to do this for CUDA as well.
IF(TARGET_BUILD_PLATFORM STREQUAL "linux")
	TARGET_COMPILE_FEATURES(PhysXSimulationControllerGpu PRIVATE cuda_std_11)
ENDIF()

TARGET_COMPILE_OPTIONS(PhysXSimulationControllerGpu PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:${ARCH_CODE_LIST}>)

IF(PX_GENERATE_SOURCE_DISTRO)
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYXGPU_SIMCONTROLLER_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYXGPU_SIMCONTROLLER_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYXGPU_SIMCONTROLLER_CUDA_INCLUDE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYXGPU_SIMCONTROLLER_CUDA_KERNELS})
ENDIF()

# enable -fPIC so we can link static libs with the editor
SET_TARGET_PROPERTIES(PhysXSimulationControllerGpu PROPERTIES
	POSITION_INDEPENDENT_CODE TRUE
)

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
# Build PhysXSolverGpu common
#

SET(PHYSX_SOURCE_DIR ${PHYSX_ROOT_DIR}/source)
SET(GPUSOLVER_SOURCE_DIR ${PHYSX_SOURCE_DIR}/gpusolver/src)

# Include here after the directories are defined so that the platform specific file can use the variables.
include(${PHYSX_ROOT_DIR}/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/PhysXSolverGpu.cmake)


# setup grouping
# solver
SET(PHYXGPU_SOLVER_HEADERS
	${PHYSX_SOURCE_DIR}/gpusolver/include/PxgConstraint.h
	${PHYSX_SOURCE_DIR}/gpusolver/include/PxgConstraintBlock.h
	${PHYSX_SOURCE_DIR}/gpusolver/include/PxgConstraintHelper.h
	${PHYSX_SOURCE_DIR}/gpusolver/include/PxgConstraintPartition.h
	${PHYSX_SOURCE_DIR}/gpusolver/include/PxgEdgeType.h
	${PHYSX_SOURCE_DIR}/gpusolver/include/PxgConstraintPrep.h
	${PHYSX_SOURCE_DIR}/gpusolver/include/PxgConstraintWriteBack.h
	${PHYSX_SOURCE_DIR}/gpusolver/include/PxgCudaSolverCore.h
	${PHYSX_SOURCE_DIR}/gpusolver/include/PxgTGSCudaSolverCore.h
	${PHYSX_SOURCE_DIR}/gpusolver/include/PxgD6Joint.h
	${PHYSX_SOURCE_DIR}/gpusolver/include/PxgD6JointData.h
	${PHYSX_SOURCE_DIR}/gpusolver/include/PxgD6JointLimit.h
	${PHYSX_SOURCE_DIR}/gpusolver/include/PxgContext.h
	${PHYSX_SOURCE_DIR}/gpusolver/include/PxgDynamicsConfiguration.h
	${PHYSX_SOURCE_DIR}/gpusolver/include/PxgDynamicsContext.h
	${PHYSX_SOURCE_DIR}/gpusolver/include/PxgTGSDynamicsContext.h
	${PHYSX_SOURCE_DIR}/gpusolver/include/PxgFrictionPatch.h
	${PHYSX_SOURCE_DIR}/gpusolver/include/PxgIslandContext.h
	${PHYSX_SOURCE_DIR}/gpusolver/include/PxgPartitionNode.h
	${PHYSX_SOURCE_DIR}/gpusolver/include/PxgSolverBody.h
	${PHYSX_SOURCE_DIR}/gpusolver/include/PxgSolverConstraint1D.h
	${PHYSX_SOURCE_DIR}/gpusolver/include/PxgSolverConstraintBlock1D.h
	${PHYSX_SOURCE_DIR}/gpusolver/include/PxgSolverConstraintDesc.h
	${PHYSX_SOURCE_DIR}/gpusolver/include/PxgSolverContext.h
	${PHYSX_SOURCE_DIR}/gpusolver/include/PxgSolverCore.h
	${PHYSX_SOURCE_DIR}/gpusolver/include/PxgSolverCoreDesc.h
    ${PHYSX_SOURCE_DIR}/gpusolver/include/PxgSolverFlags.h
	${PHYSX_SOURCE_DIR}/gpusolver/include/PxgSolverKernelIndices.h
	${PHYSX_SOURCE_DIR}/gpusolver/include/PxgSolver.h
)
SOURCE_GROUP("solver include" FILES ${PHYXGPU_SOLVER_HEADERS})

SET(PHYXGPU_SOLVER_CUDA_KERNELS
	${GPUSOLVER_SOURCE_DIR}/CUDA/solverMultiBlock.cu
	${GPUSOLVER_SOURCE_DIR}/CUDA/solverMultiBlockTGS.cu
	${GPUSOLVER_SOURCE_DIR}/CUDA/accumulateThresholdStream.cu
	${GPUSOLVER_SOURCE_DIR}/CUDA/constraintBlockPrep.cu
	${GPUSOLVER_SOURCE_DIR}/CUDA/constraintBlockPrePrep.cu
	${GPUSOLVER_SOURCE_DIR}/CUDA/artiConstraintPrep2.cu
	${GPUSOLVER_SOURCE_DIR}/CUDA/integration.cu
	${GPUSOLVER_SOURCE_DIR}/CUDA/integrationTGS.cu
	${GPUSOLVER_SOURCE_DIR}/CUDA/preIntegration.cu
	${GPUSOLVER_SOURCE_DIR}/CUDA/preIntegrationTGS.cu
	${GPUSOLVER_SOURCE_DIR}/CUDA/solver.cu
	${GPUSOLVER_SOURCE_DIR}/CUDA/constraintBlockPrepTGS.cu
)
SOURCE_GROUP("solver kernels/CUDA" FILES ${PHYXGPU_SOLVER_CUDA_KERNELS})

SET(PHYXGPU_SOLVER_CUDA_INCLUDE
	${GPUSOLVER_SOURCE_DIR}/CUDA/jointConstraintBlockPrep.cuh
    ${GPUSOLVER_SOURCE_DIR}/CUDA/jointConstraintBlockPrepTGS.cuh
	${GPUSOLVER_SOURCE_DIR}/CUDA/constant.cuh
	${GPUSOLVER_SOURCE_DIR}/CUDA/contactConstraintBlockPrep.cuh
	${GPUSOLVER_SOURCE_DIR}/CUDA/contactConstraintPrep.cuh
	${GPUSOLVER_SOURCE_DIR}/CUDA/constraintPrepShared.cuh
	${GPUSOLVER_SOURCE_DIR}/CUDA/solverBlock.cuh
	${GPUSOLVER_SOURCE_DIR}/CUDA/solverBlockTGS.cuh
	${GPUSOLVER_SOURCE_DIR}/CUDA/solverBlockCommon.cuh
	${GPUSOLVER_SOURCE_DIR}/CUDA/solver.cuh
	${GPUSOLVER_SOURCE_DIR}/CUDA/preIntegration.cuh
	${GPUSOLVER_SOURCE_DIR}/CUDA/integration.cuh

)
SOURCE_GROUP("solver kernels cuda include" FILES ${PHYXGPU_SOLVER_CUDA_INCLUDE})

SET(PHYXGPU_SOLVER_SOURCE
	${GPUSOLVER_SOURCE_DIR}/PxgConstraintPartition.cpp
	${GPUSOLVER_SOURCE_DIR}/PxgCudaSolverCore.cpp
	${GPUSOLVER_SOURCE_DIR}/PxgTGSCudaSolverCore.cpp
	${GPUSOLVER_SOURCE_DIR}/PxgSolverCore.cpp
	${GPUSOLVER_SOURCE_DIR}/PxgDynamicsContext.cpp
	${GPUSOLVER_SOURCE_DIR}/PxgTGSDynamicsContext.cpp
	${GPUSOLVER_SOURCE_DIR}/PxgContext.cpp
	${GPUSOLVER_SOURCE_DIR}/PxgSolver.cpp
)
SOURCE_GROUP("solver src" FILES ${PHYXGPU_SOLVER_SOURCE})

ADD_LIBRARY(PhysXSolverGpu ${PHYSXSOLVERGPU_LIBTYPE}
	# Solver
	${PHYXGPU_SOLVER_HEADERS}
	${PHYXGPU_SOLVER_CUDA_KERNELS}
	${PHYXGPU_SOLVER_CUDA_INCLUDE}
	${PHYXGPU_SOLVER_SOURCE}
)

TARGET_INCLUDE_DIRECTORIES(PhysXSolverGpu
	# PRIVATE ${CUDA_INCLUDE_DIRS}
	# PRIVATE ${PHYSXFOUNDATION_INCLUDES}
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
	PRIVATE ${PHYSX_SOURCE_DIR}/gpuarticulation/include
	PRIVATE ${PHYSX_SOURCE_DIR}/gpuarticulation/src/CUDA
	PRIVATE ${PHYSX_SOURCE_DIR}/cudamanager/include
)

SET_TARGET_PROPERTIES(PhysXSolverGpu PROPERTIES
	OUTPUT_NAME PhysXSolverGpu
)


IF(PHYSXSOLVERGPU_LIBTYPE STREQUAL "STATIC")
	SET_TARGET_PROPERTIES(PhysXSolverGpu PROPERTIES
		ARCHIVE_OUTPUT_NAME_DEBUG "PhysXSolverGpu_static"
		ARCHIVE_OUTPUT_NAME_CHECKED "PhysXSolverGpu_static"
		ARCHIVE_OUTPUT_NAME_PROFILE "PhysXSolverGpu_static"
		ARCHIVE_OUTPUT_NAME_RELEASE "PhysXSolverGpu_static"
	)
ENDIF()

IF(PHYSXSOLVERGPU_COMPILE_PDB_NAME_DEBUG)
	SET_TARGET_PROPERTIES(PhysXSolverGpu PROPERTIES
		COMPILE_PDB_NAME_DEBUG "${PHYSXSOLVERGPU_COMPILE_PDB_NAME_DEBUG}"
		COMPILE_PDB_NAME_CHECKED "${PHYSXSOLVERGPU_COMPILE_PDB_NAME_CHECKED}"
		COMPILE_PDB_NAME_PROFILE "${PHYSXSOLVERGPU_COMPILE_PDB_NAME_PROFILE}"
		COMPILE_PDB_NAME_RELEASE "${PHYSXSOLVERGPU_COMPILE_PDB_NAME_RELEASE}"
	)
ENDIF()


TARGET_COMPILE_DEFINITIONS(PhysXSolverGpu
	PRIVATE ${PHYSXSOLVERGPU_COMPILE_DEFS}
)

# Since we are setting the C++ standard explicitly for Linux
# we need to do this for CUDA as well.
IF(TARGET_BUILD_PLATFORM STREQUAL "linux")
	TARGET_COMPILE_FEATURES(PhysXSolverGpu PRIVATE cuda_std_11)
ENDIF()

TARGET_COMPILE_OPTIONS(PhysXSolverGpu PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:${ARCH_CODE_LIST}>)

IF(PX_GENERATE_SOURCE_DISTRO)
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYXGPU_SOLVER_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYXGPU_SOLVER_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYXGPU_SOLVER_CUDA_INCLUDE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYXGPU_SOLVER_CUDA_KERNELS})
ENDIF()

# enable -fPIC so we can link static libs with the editor
SET_TARGET_PROPERTIES(PhysXSolverGpu PROPERTIES
	POSITION_INDEPENDENT_CODE TRUE
)


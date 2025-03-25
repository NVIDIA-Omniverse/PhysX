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
# Build PhysXGpuDependencies
#

SET(PHYSX_SOURCE_DIR ${PHYSX_ROOT_DIR}/source)

# Include here after the directories are defined so that the platform specific file can use the variables.
include(${PHYSX_ROOT_DIR}/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/PhysXGpuDependencies.cmake)

SET(GPUDEPENDENCIES_INCLUDES
	${PHYSX_SOURCE_DIR}/lowlevel/common/include/pipeline/PxcNpMemBlockPool.h
	${PHYSX_SOURCE_DIR}/lowlevel/common/include/pipeline/PxcNpCacheStreamPair.h
	${PHYSX_SOURCE_DIR}/lowlevel/common/include/pipeline/PxcNpThreadContext.h
	${PHYSX_SOURCE_DIR}/lowlevelaabb/include/BpFiltering.h
	${PHYSX_ROOT_DIR}/include/common/PxRenderOutput.h
	${PHYSX_SOURCE_DIR}/geomutils/include/GuBounds.h
	${PHYSX_SOURCE_DIR}/geomutils/include/GuConvexSupport.h
	${PHYSX_SOURCE_DIR}/geomutils/include/GuRefGjkEpa.h
)
SOURCE_GROUP("Includes" FILES ${GPUDEPENDENCIES_INCLUDES})

SET(GPUDEPENDENCIES_SOURCE
	${PHYSX_SOURCE_DIR}/lowlevel/common/src/pipeline/PxcNpMemBlockPool.cpp
	${PHYSX_SOURCE_DIR}/lowlevel/common/src/pipeline/PxcNpThreadContext.cpp
	${PHYSX_SOURCE_DIR}/lowlevel/common/src/pipeline/PxcNpCacheStreamPair.cpp
	${PHYSX_SOURCE_DIR}/lowlevelaabb/src/BpFiltering.cpp
	${PHYSX_SOURCE_DIR}/lowlevelaabb/src/BpBroadPhaseUpdate.cpp
	${PHYSX_SOURCE_DIR}/lowlevelaabb/src/BpAABBManagerBase.cpp
	${PHYSX_SOURCE_DIR}/geomutils/src/GuBounds.cpp
	${PHYSX_SOURCE_DIR}/geomutils/src/convex/GuConvexUtilsInternal.cpp
	${PHYSX_SOURCE_DIR}/geomutils/src/mesh/GuTetrahedronMeshUtils.cpp
	${PHYSX_SOURCE_DIR}/geomutils/src/GuConvexGeometry.cpp
)
SOURCE_GROUP("Source" FILES ${GPUDEPENDENCIES_SOURCE})

SET(PHYSXFOUNDATION_SOURCE
	${PHYSX_SOURCE_DIR}/foundation/FdAllocator.cpp
	${PHYSX_SOURCE_DIR}/foundation/FdString.cpp
	${PHYSX_SOURCE_DIR}/foundation/FdTempAllocator.cpp
	${PHYSX_SOURCE_DIR}/foundation/FdAssert.cpp
	${PHYSX_SOURCE_DIR}/foundation/FdMathUtils.cpp
	${PHYSX_SOURCE_DIR}/foundation/FdFoundation.cpp
)
SOURCE_GROUP("foundation\\src" FILES ${PHYSXFOUNDATION_SOURCE})

ADD_LIBRARY(PhysXGpuDependencies OBJECT
	${GPUDEPENDENCIES_INCLUDES}
	${GPUDEPENDENCIES_SOURCE}
	${GPUDEPENDENCIES_PLATFORM_SOURCE}
	${PHYSXFOUNDATION_SOURCE}
)

TARGET_INCLUDE_DIRECTORIES(PhysXGpuDependencies
	PRIVATE ${GPUDEPENDENCIES_PLATFORM_INCLUDES}
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
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/convex
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/hf
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/common
	PRIVATE ${PHYSX_SOURCE_DIR}/gpucommon/include
	PRIVATE ${PHYSX_SOURCE_DIR}/gpucommon/src/CUDA
)

TARGET_COMPILE_DEFINITIONS(PhysXGpuDependencies
	PRIVATE ${GPUDEPENDENCIES_COMPILE_DEFS}
)

IF(PX_GENERATE_SOURCE_DISTRO)
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${GPUDEPENDENCIES_INCLUDES})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${GPUDEPENDENCIES_SOURCE})
ENDIF()

## SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
## SPDX-License-Identifier: Apache-2.0

#
# Build PhysXFoundation common
#

SET(PHYSX_SOURCE_DIR ${PHYSX_ROOT_DIR}/source)
SET(LL_SOURCE_DIR ${PHYSX_SOURCE_DIR}/foundation)

# Include here after the directories are defined so that the platform specific file can use the variables.
include(${PHYSX_ROOT_DIR}/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/PhysXFoundation.cmake)

SET(PHYSXFOUNDATION_HEADERS
	${PHYSX_ROOT_DIR}/include/foundation/PxFoundation.h
	${PHYSX_ROOT_DIR}/include/foundation/PxAssert.h
	${PHYSX_ROOT_DIR}/include/foundation/PxFoundationConfig.h
	${PHYSX_ROOT_DIR}/include/foundation/PxMathUtils.h
	${PHYSX_ROOT_DIR}/include/foundation/PxAlignedMalloc.h
	${PHYSX_ROOT_DIR}/include/foundation/PxAllocatorCallback.h
	${PHYSX_ROOT_DIR}/include/foundation/PxProfiler.h
	${PHYSX_ROOT_DIR}/include/foundation/PxAoS.h
	${PHYSX_ROOT_DIR}/include/foundation/PxAlloca.h
	${PHYSX_ROOT_DIR}/include/foundation/PxAllocator.h
	${PHYSX_ROOT_DIR}/include/foundation/PxArray.h
	${PHYSX_ROOT_DIR}/include/foundation/PxAtomic.h
	${PHYSX_ROOT_DIR}/include/foundation/PxBasicTemplates.h
	${PHYSX_ROOT_DIR}/include/foundation/PxBitMap.h
	${PHYSX_ROOT_DIR}/include/foundation/PxBitAndData.h	
	${PHYSX_ROOT_DIR}/include/foundation/PxBitUtils.h
	${PHYSX_ROOT_DIR}/include/foundation/PxBounds3.h
	${PHYSX_ROOT_DIR}/include/foundation/PxBroadcast.h
	${PHYSX_ROOT_DIR}/include/foundation/PxConstructor.h
	${PHYSX_ROOT_DIR}/include/foundation/PxErrorCallback.h
	${PHYSX_ROOT_DIR}/include/foundation/PxErrors.h
	${PHYSX_ROOT_DIR}/include/foundation/PxFlags.h
	${PHYSX_ROOT_DIR}/include/foundation/PxFPU.h
	${PHYSX_ROOT_DIR}/include/foundation/PxInlineAoS.h
	${PHYSX_ROOT_DIR}/include/foundation/PxIntrinsics.h
	${PHYSX_ROOT_DIR}/include/foundation/PxHash.h
	${PHYSX_ROOT_DIR}/include/foundation/PxHashInternals.h
	${PHYSX_ROOT_DIR}/include/foundation/PxHashMap.h
	${PHYSX_ROOT_DIR}/include/foundation/PxHashSet.h
	${PHYSX_ROOT_DIR}/include/foundation/PxInlineAllocator.h
	${PHYSX_ROOT_DIR}/include/foundation/PxInlineArray.h
	${PHYSX_ROOT_DIR}/include/foundation/PxMathIntrinsics.h
	${PHYSX_ROOT_DIR}/include/foundation/PxMutex.h
	${PHYSX_ROOT_DIR}/include/foundation/PxIO.h
	${PHYSX_ROOT_DIR}/include/foundation/PxMat33.h
	${PHYSX_ROOT_DIR}/include/foundation/PxMat34.h
	${PHYSX_ROOT_DIR}/include/foundation/PxMat44.h
	${PHYSX_ROOT_DIR}/include/foundation/PxMath.h	
	${PHYSX_ROOT_DIR}/include/foundation/PxMemory.h
	${PHYSX_ROOT_DIR}/include/foundation/PxPlane.h
	${PHYSX_ROOT_DIR}/include/foundation/PxPool.h
	${PHYSX_ROOT_DIR}/include/foundation/PxPreprocessor.h
	${PHYSX_ROOT_DIR}/include/foundation/PxQuat.h
	${PHYSX_ROOT_DIR}/include/foundation/PxPhysicsVersion.h
	${PHYSX_ROOT_DIR}/include/foundation/PxSortInternals.h
	${PHYSX_ROOT_DIR}/include/foundation/PxSimpleTypes.h
	${PHYSX_ROOT_DIR}/include/foundation/PxSList.h
	${PHYSX_ROOT_DIR}/include/foundation/PxSocket.h
	${PHYSX_ROOT_DIR}/include/foundation/PxSort.h
	${PHYSX_ROOT_DIR}/include/foundation/PxStrideIterator.h
	${PHYSX_ROOT_DIR}/include/foundation/PxString.h
	${PHYSX_ROOT_DIR}/include/foundation/PxSync.h
	${PHYSX_ROOT_DIR}/include/foundation/PxTempAllocator.h
	${PHYSX_ROOT_DIR}/include/foundation/PxThread.h
	${PHYSX_ROOT_DIR}/include/foundation/PxTransform.h
	${PHYSX_ROOT_DIR}/include/foundation/PxTime.h
	${PHYSX_ROOT_DIR}/include/foundation/PxUnionCast.h
	${PHYSX_ROOT_DIR}/include/foundation/PxUserAllocated.h
	${PHYSX_ROOT_DIR}/include/foundation/PxUtilities.h
	${PHYSX_ROOT_DIR}/include/foundation/PxVec2.h
	${PHYSX_ROOT_DIR}/include/foundation/PxVec3.h
	${PHYSX_ROOT_DIR}/include/foundation/PxVec4.h
	${PHYSX_ROOT_DIR}/include/foundation/PxVecMath.h
	${PHYSX_ROOT_DIR}/include/foundation/PxVecMathAoSScalar.h
	${PHYSX_ROOT_DIR}/include/foundation/PxVecMathAoSScalarInline.h
	${PHYSX_ROOT_DIR}/include/foundation/PxVecMathSSE.h
	${PHYSX_ROOT_DIR}/include/foundation/PxVecQuat.h
	${PHYSX_ROOT_DIR}/include/foundation/PxVecTransform.h
	${PHYSX_ROOT_DIR}/include/foundation/PxSIMDHelpers.h
)
SOURCE_GROUP(include FILES ${PHYSXFOUNDATION_HEADERS})


SET(PHYSXFOUNDATION_SOURCE
	${LL_SOURCE_DIR}/FdAllocator.cpp
	${LL_SOURCE_DIR}/FdString.cpp
	${LL_SOURCE_DIR}/FdTempAllocator.cpp
	${LL_SOURCE_DIR}/FdAssert.cpp
	${LL_SOURCE_DIR}/FdMathUtils.cpp
	${LL_SOURCE_DIR}/FdFoundation.cpp
	${LL_SOURCE_DIR}/FdFoundation.h
)
SOURCE_GROUP(src FILES ${PHYSXFOUNDATION_SOURCE})

ADD_LIBRARY(PhysXFoundation ${PHYSXFOUNDATION_LIBTYPE} 
	${PHYSXFOUNDATION_HEADERS}
	${PHYSXFOUNDATION_SOURCE}
	${PHYSXFOUNDATION_PLATFORM_FILES}
)

# Add the headers to the install
IF(NOT DEFINED PX_ENABLE_INSTALL OR PX_ENABLE_INSTALL)
	INSTALL(FILES ${PHYSXFOUNDATION_HEADERS} DESTINATION include/foundation)
ENDIF()

TARGET_INCLUDE_DIRECTORIES(PhysXFoundation 
	PUBLIC 
		$<BUILD_INTERFACE:${PHYSX_ROOT_DIR}/include>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    
	PRIVATE ${PHYSXFOUNDATION_PLATFORM_INCLUDES}
)

TARGET_COMPILE_DEFINITIONS(PhysXFoundation 
	PRIVATE ${PHYSXFOUNDATION_COMPILE_DEFS}
)

SET_TARGET_PROPERTIES(PhysXFoundation PROPERTIES
	OUTPUT_NAME PhysXFoundation
)

IF(PHYSXFOUNDATION_LIBTYPE STREQUAL "STATIC")	
	SET_TARGET_PROPERTIES(PhysXFoundation PROPERTIES 			
		ARCHIVE_OUTPUT_NAME_DEBUG "PhysXFoundation_static"
		ARCHIVE_OUTPUT_NAME_CHECKED "PhysXFoundation_static"
		ARCHIVE_OUTPUT_NAME_PROFILE "PhysXFoundation_static"
		ARCHIVE_OUTPUT_NAME_RELEASE "PhysXFoundation_static"
	)	
ENDIF()

IF(PHYSXFOUNDATION_COMPILE_PDB_NAME_DEBUG)
	SET_TARGET_PROPERTIES(PhysXFoundation PROPERTIES 
		COMPILE_PDB_NAME_DEBUG ${PHYSXFOUNDATION_COMPILE_PDB_NAME_DEBUG}
		COMPILE_PDB_NAME_CHECKED ${PHYSXFOUNDATION_COMPILE_PDB_NAME_CHECKED}
		COMPILE_PDB_NAME_PROFILE ${PHYSXFOUNDATION_COMPILE_PDB_NAME_PROFILE}
		COMPILE_PDB_NAME_RELEASE ${PHYSXFOUNDATION_COMPILE_PDB_NAME_RELEASE}
	)
ENDIF()

# Add linked libraries
TARGET_LINK_LIBRARIES(PhysXFoundation 
	PRIVATE ${PHYSXFOUNDATION_PLATFORM_LINKED_LIBS}
)

IF(PX_GENERATE_SOURCE_DISTRO)		
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSXFOUNDATION_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSXFOUNDATION_SOURCE_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSXFOUNDATION_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSXFOUNDATION_PLATFORM_FILES})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PXSHARED_PLATFORM_HEADERS})
ENDIF()

# enable -fPIC so we can link static libs with the editor
SET_TARGET_PROPERTIES(PhysXFoundation PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

IF(PLATFORM_COMPILE_FLAGS)
	SET_TARGET_PROPERTIES(PhysXFoundation PROPERTIES COMPILE_FLAGS ${PLATFORM_COMPILE_FLAGS})
ENDIF()

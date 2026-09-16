## SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
## SPDX-License-Identifier: Apache-2.0

#
# Build PhysXPvdSDK common
#

SET(PHYSX_SOURCE_DIR ${PHYSX_ROOT_DIR}/source)
SET(LL_SOURCE_DIR ${PHYSX_SOURCE_DIR}/pvd)

# Include here after the directories are defined so that the platform specific file can use the variables.
include(${PHYSX_ROOT_DIR}/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/PhysXPvdSDK.cmake)


SET(PHYSXPVDSDK_HEADERS
	${PHYSX_ROOT_DIR}/include/pvd/PxPvd.h
	${PHYSX_ROOT_DIR}/include/pvd/PxPvdTransport.h
)
SOURCE_GROUP(include FILES ${PHYSXPVDSDK_HEADERS})

SET(PHYSXPVDSDK_SOURCE
	${LL_SOURCE_DIR}/src/PsFileBuffer.h
	${LL_SOURCE_DIR}/src/PxFileBuf.h
	${LL_SOURCE_DIR}/src/PxProfileContextProvider.h
	${LL_SOURCE_DIR}/src/PxProfileContextProviderImpl.h
	${LL_SOURCE_DIR}/src/PxProfileDataBuffer.h
	${LL_SOURCE_DIR}/src/PxProfileDataParsing.h
	${LL_SOURCE_DIR}/src/PxProfileEventBuffer.h
	${LL_SOURCE_DIR}/src/PxProfileEventBufferAtomic.h
	${LL_SOURCE_DIR}/src/PxProfileEventBufferClient.h
	${LL_SOURCE_DIR}/src/PxProfileEventBufferClientManager.h
	${LL_SOURCE_DIR}/src/PxProfileEventId.h
	${LL_SOURCE_DIR}/src/PxProfileEventImpl.cpp
	${LL_SOURCE_DIR}/src/PxProfileEventMutex.h
	${LL_SOURCE_DIR}/src/PxProfileEventNames.h
	${LL_SOURCE_DIR}/src/PxProfileEvents.h
	${LL_SOURCE_DIR}/src/PxProfileEventSender.h
	${LL_SOURCE_DIR}/src/PxProfileEventSerialization.h
	${LL_SOURCE_DIR}/src/PxProfileMemory.h
	${LL_SOURCE_DIR}/src/PxProfileMemoryBuffer.h
	${LL_SOURCE_DIR}/src/PxProfileMemoryEventBuffer.h
	${LL_SOURCE_DIR}/src/PxProfileMemoryEvents.h
	${LL_SOURCE_DIR}/src/PxProfileScopedEvent.h
	${LL_SOURCE_DIR}/src/PxProfileScopedMutexLock.h
	${LL_SOURCE_DIR}/src/PxPvdProfileZone.h
	${LL_SOURCE_DIR}/src/PxProfileZoneImpl.h
	${LL_SOURCE_DIR}/src/PxProfileZoneManager.h
	${LL_SOURCE_DIR}/src/PxProfileZoneManagerImpl.h
	${LL_SOURCE_DIR}/src/PxPvd.cpp
	${LL_SOURCE_DIR}/src/PxPvdBits.h
	${LL_SOURCE_DIR}/src/PxPvdByteStreams.h
	${LL_SOURCE_DIR}/src/PxPvdCommStreamEvents.h
	${LL_SOURCE_DIR}/src/PxPvdCommStreamEventSink.h
	${LL_SOURCE_DIR}/src/PxPvdCommStreamTypes.h
	${LL_SOURCE_DIR}/src/PxPvdDataStream.cpp
	${LL_SOURCE_DIR}/src/PxPvdDefaultFileTransport.cpp
	${LL_SOURCE_DIR}/src/PxPvdDefaultFileTransport.h
	${LL_SOURCE_DIR}/src/PxPvdDefaultSocketTransport.cpp
	${LL_SOURCE_DIR}/src/PxPvdDefaultSocketTransport.h
	${LL_SOURCE_DIR}/src/PxPvdFoundation.h
	${LL_SOURCE_DIR}/src/PxPvdImpl.cpp
	${LL_SOURCE_DIR}/src/PxPvdImpl.h
	${LL_SOURCE_DIR}/src/PxPvdInternalByteStreams.h
	${LL_SOURCE_DIR}/src/PxPvdMarshalling.h
	${LL_SOURCE_DIR}/src/PxPvdMemClient.cpp
	${LL_SOURCE_DIR}/src/PxPvdMemClient.h
	${LL_SOURCE_DIR}/src/PxPvdObjectModelInternalTypeDefs.h
	${LL_SOURCE_DIR}/src/PxPvdObjectModelInternalTypes.h
	${LL_SOURCE_DIR}/src/PxPvdObjectModelMetaData.cpp
	${LL_SOURCE_DIR}/src/PxPvdObjectModelMetaData.h
	${LL_SOURCE_DIR}/src/PxPvdObjectRegistrar.cpp
	${LL_SOURCE_DIR}/src/PxPvdObjectRegistrar.h
	${LL_SOURCE_DIR}/src/PxPvdProfileZoneClient.cpp
	${LL_SOURCE_DIR}/src/PxPvdProfileZoneClient.h
	${LL_SOURCE_DIR}/src/PxPvdUserRenderer.cpp
	${LL_SOURCE_DIR}/src/PxPvdUserRenderImpl.h
	${LL_SOURCE_DIR}/src/PxPvdUserRenderTypes.h
)
SOURCE_GROUP(src\\src FILES ${PHYSXPVDSDK_SOURCE})

SET(PHYSXPVDSDK_INTERNAL_HEADERS
	${LL_SOURCE_DIR}/include/PsPvd.h
	${LL_SOURCE_DIR}/include/PxProfileAllocatorWrapper.h
	${LL_SOURCE_DIR}/include/PxPvdClient.h
	${LL_SOURCE_DIR}/include/PxPvdDataStream.h
	${LL_SOURCE_DIR}/include/PxPvdDataStreamHelpers.h
	${LL_SOURCE_DIR}/include/PxPvdErrorCodes.h
	${LL_SOURCE_DIR}/include/PxPvdObjectModelBaseTypes.h
	${LL_SOURCE_DIR}/include/PxPvdUserRenderer.h
)
SOURCE_GROUP(src\\include FILES ${PHYSXPVDSDK_INTERNAL_HEADERS})

ADD_LIBRARY(PhysXPvdSDK ${PHYSXPVDSDK_LIBTYPE} 
	${PHYSXPVDSDK_HEADERS}

	${PHYSXPVDSDK_INTERNAL_HEADERS}
	${PHYSXPVDSDK_SOURCE}
	
	${PHYSXPVDSDK_PLATFORM_FILES}
)

TARGET_INCLUDE_DIRECTORIES(PhysXPvdSDK 
	PRIVATE ${PHYSXPVDSDK_PLATFORM_INCLUDES}
	
	PRIVATE ${PHYSX_ROOT_DIR}/include
	PRIVATE ${LL_SOURCE_DIR}/include
	PRIVATE ${PHYSX_SOURCE_DIR}/filebuf/include
	
)

TARGET_COMPILE_DEFINITIONS(PhysXPvdSDK 
	PRIVATE ${PHYSXPVDSDK_COMPILE_DEFS}
)

# Add linked libraries
IF(PX_GENERATE_STATIC_LIBRARIES)
	TARGET_LINK_LIBRARIES(PhysXPvdSDK 
		PUBLIC ${PHYSXPVDSDK_PLATFORM_LINKED_LIBS}
		PRIVATE PhysX PhysXFoundation
	)
ELSE()
	TARGET_LINK_LIBRARIES(PhysXPvdSDK 
		PUBLIC ${PHYSXPVDSDK_PLATFORM_LINKED_LIBS}
		PRIVATE PhysXFoundation
	)
ENDIF()

SET_TARGET_PROPERTIES(PhysXPvdSDK PROPERTIES
	OUTPUT_NAME PhysXPvdSDK
)

IF(PHYSXPVDSDK_LIBTYPE STREQUAL "STATIC")	
	SET_TARGET_PROPERTIES(PhysXPvdSDK PROPERTIES 
		ARCHIVE_OUTPUT_NAME_DEBUG "PhysXPvdSDK_static"
		ARCHIVE_OUTPUT_NAME_CHECKED "PhysXPvdSDK_static"
		ARCHIVE_OUTPUT_NAME_PROFILE "PhysXPvdSDK_static"
		ARCHIVE_OUTPUT_NAME_RELEASE "PhysXPvdSDK_static"
	)	
ENDIF()

IF(PHYSXPVDSDK_COMPILE_PDB_NAME_DEBUG)
	SET_TARGET_PROPERTIES(PhysXPvdSDK PROPERTIES 
		COMPILE_PDB_NAME_DEBUG ${PHYSXPVDSDK_COMPILE_PDB_NAME_DEBUG}
		COMPILE_PDB_NAME_CHECKED ${PHYSXPVDSDK_COMPILE_PDB_NAME_CHECKED}
		COMPILE_PDB_NAME_PROFILE ${PHYSXPVDSDK_COMPILE_PDB_NAME_PROFILE}
		COMPILE_PDB_NAME_RELEASE ${PHYSXPVDSDK_COMPILE_PDB_NAME_RELEASE}
	)
ENDIF()

IF(PX_GENERATE_SOURCE_DISTRO)		
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSXPVDSDK_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSXPVDSDK_INTERNAL_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSXPVDSDK_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSXPVDSDK_PLATFORM_FILES})
ENDIF()

# enable -fPIC so we can link static libs with the editor
SET_TARGET_PROPERTIES(PhysXPvdSDK PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

## SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
## SPDX-License-Identifier: Apache-2.0

#
# Build PhysXCharacterKinematic common
#

SET(PHYSX_SOURCE_DIR ${PHYSX_ROOT_DIR}/source)
SET(LL_SOURCE_DIR ${PHYSX_SOURCE_DIR}/physxcharacterkinematic/src)

# Include here after the directories are defined so that the platform specific file can use the variables.
include(${PHYSX_ROOT_DIR}/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/PhysXCharacterKinematic.cmake)


SET(PHYSXCCT_HEADERS
	${PHYSX_ROOT_DIR}/include/characterkinematic/PxBoxController.h
	${PHYSX_ROOT_DIR}/include/characterkinematic/PxCapsuleController.h
	${PHYSX_ROOT_DIR}/include/characterkinematic/PxController.h
	${PHYSX_ROOT_DIR}/include/characterkinematic/PxControllerBehavior.h
	${PHYSX_ROOT_DIR}/include/characterkinematic/PxControllerManager.h
	${PHYSX_ROOT_DIR}/include/characterkinematic/PxControllerObstacles.h
	${PHYSX_ROOT_DIR}/include/characterkinematic/PxExtended.h
)
SOURCE_GROUP(include FILES ${PHYSXCCT_HEADERS})

SET(PHYSXCCT_SOURCE
	${LL_SOURCE_DIR}/CctBoxController.cpp
	${LL_SOURCE_DIR}/CctCapsuleController.cpp
	${LL_SOURCE_DIR}/CctCharacterController.cpp
	${LL_SOURCE_DIR}/CctCharacterControllerCallbacks.cpp
	${LL_SOURCE_DIR}/CctCharacterControllerManager.cpp
	${LL_SOURCE_DIR}/CctController.cpp
	${LL_SOURCE_DIR}/CctObstacleContext.cpp
	${LL_SOURCE_DIR}/CctSweptBox.cpp
	${LL_SOURCE_DIR}/CctSweptCapsule.cpp
	${LL_SOURCE_DIR}/CctSweptVolume.cpp
	${LL_SOURCE_DIR}/CctBoxController.h
	${LL_SOURCE_DIR}/CctCapsuleController.h
	${LL_SOURCE_DIR}/CctCharacterController.h
	${LL_SOURCE_DIR}/CctCharacterControllerManager.h
	${LL_SOURCE_DIR}/CctController.h
	${LL_SOURCE_DIR}/CctInternalStructs.h
	${LL_SOURCE_DIR}/CctObstacleContext.h
	${LL_SOURCE_DIR}/CctSweptBox.h
	${LL_SOURCE_DIR}/CctSweptCapsule.h
	${LL_SOURCE_DIR}/CctSweptVolume.h
	${LL_SOURCE_DIR}/CctUtils.h
)
SOURCE_GROUP(src FILES ${PHYSXCCT_SOURCE})

ADD_LIBRARY(PhysXCharacterKinematic ${PHYSXCHARACTERKINEMATIC_LIBTYPE}
	${PHYSXCCT_HEADERS}
	${PHYSXCCT_SOURCE}
)

IF(NOT DEFINED PX_ENABLE_INSTALL OR PX_ENABLE_INSTALL)
	INSTALL(FILES ${PHYSXCCT_HEADERS} DESTINATION include/characterkinematic)
ENDIF()

TARGET_INCLUDE_DIRECTORIES(PhysXCharacterKinematic 

	PRIVATE ${PHYSXCHARACTERKINEMATICS_PLATFORM_INCLUDES}

	PRIVATE ${PHYSX_ROOT_DIR}/include
	
	PRIVATE ${PHYSX_SOURCE_DIR}/common/src
	
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/include
)

TARGET_COMPILE_DEFINITIONS(PhysXCharacterKinematic 

	# Common to all configurations
	PRIVATE ${PHYSXCHARACTERKINEMATICS_COMPILE_DEFS}
)

SET_TARGET_PROPERTIES(PhysXCharacterKinematic PROPERTIES
	OUTPUT_NAME PhysXCharacterKinematic
)


IF(PHYSXCHARACTERKINEMATIC_LIBTYPE STREQUAL "STATIC")	
	SET_TARGET_PROPERTIES(PhysXCharacterKinematic PROPERTIES 
		ARCHIVE_OUTPUT_NAME_DEBUG "PhysXCharacterKinematic_static"
		ARCHIVE_OUTPUT_NAME_CHECKED "PhysXCharacterKinematic_static"
		ARCHIVE_OUTPUT_NAME_PROFILE "PhysXCharacterKinematic_static"
		ARCHIVE_OUTPUT_NAME_RELEASE "PhysXCharacterKinematic_static"
	)
ENDIF()

IF(PHYSXCHARACTERKINEMATIC_COMPILE_PDB_NAME_DEBUG)
	SET_TARGET_PROPERTIES(PhysXCharacterKinematic PROPERTIES 
		COMPILE_PDB_NAME_DEBUG ${PHYSXCHARACTERKINEMATIC_COMPILE_PDB_NAME_DEBUG}
		COMPILE_PDB_NAME_CHECKED ${PHYSXCHARACTERKINEMATIC_COMPILE_PDB_NAME_CHECKED}
		COMPILE_PDB_NAME_PROFILE ${PHYSXCHARACTERKINEMATIC_COMPILE_PDB_NAME_PROFILE}
		COMPILE_PDB_NAME_RELEASE ${PHYSXCHARACTERKINEMATIC_COMPILE_PDB_NAME_RELEASE}
	)
ENDIF()

TARGET_LINK_LIBRARIES(PhysXCharacterKinematic
	PUBLIC ${PHYSXCHARACTERKINEMATICS_PLATFORM_LINKED_LIBS} 
	PUBLIC PhysXFoundation
)


IF(PX_GENERATE_SOURCE_DISTRO)
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSXCCT_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSXCCT_SOURCE})
ENDIF()

# enable -fPIC so we can link static libs with the editor
SET_TARGET_PROPERTIES(PhysXCharacterKinematic PROPERTIES POSITION_INDEPENDENT_CODE TRUE)


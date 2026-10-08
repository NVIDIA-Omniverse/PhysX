## SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
## SPDX-License-Identifier: Apache-2.0

#
# Build SnippetUtils common
#

# Include here after the directories are defined so that the platform specific file can use the variables.
INCLUDE(${PHYSX_ROOT_DIR}/snippets/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/SnippetUtils.cmake)

SET(SNIPPETUTILS_FILES
	${PHYSX_ROOT_DIR}/snippets/snippetutils/SnippetUtils.cpp
	${PHYSX_ROOT_DIR}/snippets/snippetutils/SnippetUtils.h
	${PHYSX_ROOT_DIR}/snippets/snippetutils/SnippetImmUtils.cpp
	${PHYSX_ROOT_DIR}/snippets/snippetutils/SnippetImmUtils.h
)

ADD_LIBRARY(SnippetUtils STATIC
	${SNIPPETUTILS_FILES}
)

TARGET_INCLUDE_DIRECTORIES(SnippetUtils
	PRIVATE ${SNIPPETUTILS_PLATFORM_INCLUDES}
	
	PRIVATE ${PHYSX_ROOT_DIR}/include
	PRIVATE ${PHYSX_ROOT_DIR}/source/common/src
)

TARGET_COMPILE_DEFINITIONS(SnippetUtils 
	PRIVATE ${SNIPPETUTILS_COMPILE_DEFS}
)


SET_TARGET_PROPERTIES(SnippetUtils PROPERTIES 
    COMPILE_PDB_NAME_DEBUG "SnippetUtils_static_${CMAKE_DEBUG_POSTFIX}"
    COMPILE_PDB_NAME_CHECKED "SnippetUtils_static_${CMAKE_CHECKED_POSTFIX}"
    COMPILE_PDB_NAME_PROFILE "SnippetUtils_static_${CMAKE_PROFILE_POSTFIX}"
    COMPILE_PDB_NAME_RELEASE "SnippetUtils_static_${CMAKE_RELEASE_POSTFIX}"

    ARCHIVE_OUTPUT_NAME_DEBUG "SnippetUtils_static"
    ARCHIVE_OUTPUT_NAME_CHECKED "SnippetUtils_static"
    ARCHIVE_OUTPUT_NAME_PROFILE "SnippetUtils_static"
    ARCHIVE_OUTPUT_NAME_RELEASE "SnippetUtils_static"
)


TARGET_LINK_LIBRARIES(SnippetUtils 
	PUBLIC PhysXFoundation
)	

IF(PX_GENERATE_SOURCE_DISTRO)
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${SNIPPETUTILS_FILES})	
ENDIF()

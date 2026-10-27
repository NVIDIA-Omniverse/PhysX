## SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
## SPDX-License-Identifier: Apache-2.0

#
# Build SnippetRender common
#

# Include here after the directories are defined so that the platform specific file can use the variables.
include(${PHYSX_ROOT_DIR}/snippets/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/SnippetRender.cmake)

SET(SNIPPETRENDER_FILES
	${PHYSX_ROOT_DIR}/snippets/graphics/src/glew.c
	${PHYSX_ROOT_DIR}/snippets/snippetrender/SnippetCamera.cpp
	${PHYSX_ROOT_DIR}/snippets/snippetrender/SnippetCamera.h
	${PHYSX_ROOT_DIR}/snippets/snippetrender/SnippetFontData.h
	${PHYSX_ROOT_DIR}/snippets/snippetrender/SnippetFontRenderer.h
	${PHYSX_ROOT_DIR}/snippets/snippetrender/SnippetFontRenderer.cpp
	${PHYSX_ROOT_DIR}/snippets/snippetrender/SnippetRender.cpp
	${PHYSX_ROOT_DIR}/snippets/snippetrender/SnippetRender.h
)

ADD_LIBRARY(SnippetRender STATIC
	${SNIPPETRENDER_FILES}
	${SNIPPETRENDER_PLATFORM_FILES}
)

#TODO remove all GL dependencies in SnippetRender.h interface would be nice to get rid of the PUBLIC headers
TARGET_INCLUDE_DIRECTORIES(SnippetRender
	PRIVATE ${PHYSX_ROOT_DIR}/include
	PUBLIC ${PHYSX_ROOT_DIR}/snippets/graphics/include
	PUBLIC ${SNIPPETRENDER_PLATFORM_INCLUDES}
)

TARGET_COMPILE_DEFINITIONS(SnippetRender
	PRIVATE ${SNIPPETRENDER_COMPILE_DEFS}
)

SET_TARGET_PROPERTIES(SnippetRender PROPERTIES
    COMPILE_PDB_NAME_DEBUG "SnippetRender_static_${CMAKE_DEBUG_POSTFIX}"
    COMPILE_PDB_NAME_CHECKED "SnippetRender_static_${CMAKE_CHECKED_POSTFIX}"
    COMPILE_PDB_NAME_PROFILE "SnippetRender_static_${CMAKE_PROFILE_POSTFIX}"
    COMPILE_PDB_NAME_RELEASE "SnippetRender_static_${CMAKE_RELEASE_POSTFIX}"

    ARCHIVE_OUTPUT_NAME_DEBUG "SnippetRender_static"
    ARCHIVE_OUTPUT_NAME_CHECKED "SnippetRender_static"
    ARCHIVE_OUTPUT_NAME_PROFILE "SnippetRender_static"
    ARCHIVE_OUTPUT_NAME_RELEASE "SnippetRender_static"
)

TARGET_LINK_LIBRARIES(SnippetRender
	PUBLIC PhysXFoundation
	PUBLIC ${SNIPPETRENDER_PLATFORM_LINKED_LIBS}
)

IF(PX_GENERATE_SOURCE_DISTRO)
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${SNIPPETRENDER_FILES})
ENDIF()

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
## Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.

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

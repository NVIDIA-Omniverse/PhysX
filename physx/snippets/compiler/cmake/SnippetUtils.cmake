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
## Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.

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


IF(NV_USE_GAMEWORKS_OUTPUT_DIRS)
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
ELSE()
	SET_TARGET_PROPERTIES(SnippetUtils PROPERTIES 
		COMPILE_PDB_NAME_DEBUG "SnippetUtils${CMAKE_DEBUG_POSTFIX}"
		COMPILE_PDB_NAME_CHECKED "SnippetUtils${CMAKE_CHECKED_POSTFIX}"
		COMPILE_PDB_NAME_PROFILE "SnippetUtils${CMAKE_PROFILE_POSTFIX}"
		COMPILE_PDB_NAME_RELEASE "SnippetUtils${CMAKE_RELEASE_POSTFIX}"
	)
ENDIF()


TARGET_LINK_LIBRARIES(SnippetUtils 
	PUBLIC PhysXFoundation
)	

IF(PX_GENERATE_SOURCE_DISTRO)
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${SNIPPETUTILS_FILES})	
ENDIF()

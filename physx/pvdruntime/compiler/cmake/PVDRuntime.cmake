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
# Build Server Test template
#
# Include here after the directories are defined so that the platform specific file can use the variables.

include(${PHYSX_ROOT_DIR}/pvdruntime/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/PVDRuntime.cmake)

SET(PVDRUNTIME_HEADERS
  ${PHYSX_ROOT_DIR}/pvdruntime/include/OmniPvdLibraryFunctions.h
  ${PHYSX_ROOT_DIR}/pvdruntime/include/OmniPvdCommands.h
  ${PHYSX_ROOT_DIR}/pvdruntime/include/OmniPvdDefines.h  
  ${PHYSX_ROOT_DIR}/pvdruntime/include/OmniPvdReader.h
  ${PHYSX_ROOT_DIR}/pvdruntime/include/OmniPvdWriter.h
  ${PHYSX_ROOT_DIR}/pvdruntime/include/OmniPvdReadStream.h
  ${PHYSX_ROOT_DIR}/pvdruntime/include/OmniPvdWriteStream.h
  ${PHYSX_ROOT_DIR}/pvdruntime/include/OmniPvdFileReadStream.h
  ${PHYSX_ROOT_DIR}/pvdruntime/include/OmniPvdFileWriteStream.h
  ${PHYSX_ROOT_DIR}/pvdruntime/include/OmniPvdMemoryStream.h
  ${PHYSX_ROOT_DIR}/pvdruntime/include/OmniPvdLibraryHelpers.h
  ${PHYSX_ROOT_DIR}/pvdruntime/include/OmniPvdLoader.h
)
SOURCE_GROUP(include FILES ${PVDRUNTIME_HEADERS})

SET(PVDRUNTIME_HEADERS_USER
  ${PHYSX_ROOT_DIR}/pvdruntime/include/OmniPvdLoader.h
)

INSTALL(FILES ${PVDRUNTIME_HEADERS} ${PVDRUNTIME_HEADERS_USER} DESTINATION pvdruntime/include)

SET(PVDRUNTIME_SOURCES
  ${PHYSX_ROOT_DIR}/pvdruntime/src/OmniPvdDefinesInternal.h
  ${PHYSX_ROOT_DIR}/pvdruntime/src/OmniPvdHelpers.h
  ${PHYSX_ROOT_DIR}/pvdruntime/src/OmniPvdHelpers.cpp
  ${PHYSX_ROOT_DIR}/pvdruntime/src/OmniPvdLibraryFunctionsImpl.cpp
  ${PHYSX_ROOT_DIR}/pvdruntime/src/OmniPvdLog.h
  ${PHYSX_ROOT_DIR}/pvdruntime/src/OmniPvdLog.cpp
  ${PHYSX_ROOT_DIR}/pvdruntime/src/OmniPvdReaderImpl.h
  ${PHYSX_ROOT_DIR}/pvdruntime/src/OmniPvdReaderImpl.cpp
  ${PHYSX_ROOT_DIR}/pvdruntime/src/OmniPvdWriterImpl.h
  ${PHYSX_ROOT_DIR}/pvdruntime/src/OmniPvdWriterImpl.cpp
  ${PHYSX_ROOT_DIR}/pvdruntime/src/OmniPvdFileReadStreamImpl.h
  ${PHYSX_ROOT_DIR}/pvdruntime/src/OmniPvdFileReadStreamImpl.cpp
  ${PHYSX_ROOT_DIR}/pvdruntime/src/OmniPvdFileWriteStreamImpl.h
  ${PHYSX_ROOT_DIR}/pvdruntime/src/OmniPvdFileWriteStreamImpl.cpp
  ${PHYSX_ROOT_DIR}/pvdruntime/src/OmniPvdMemoryStreamImpl.h
  ${PHYSX_ROOT_DIR}/pvdruntime/src/OmniPvdMemoryStreamImpl.cpp
  ${PHYSX_ROOT_DIR}/pvdruntime/src/OmniPvdMemoryReadStreamImpl.h
  ${PHYSX_ROOT_DIR}/pvdruntime/src/OmniPvdMemoryReadStreamImpl.cpp
  ${PHYSX_ROOT_DIR}/pvdruntime/src/OmniPvdMemoryWriteStreamImpl.h
  ${PHYSX_ROOT_DIR}/pvdruntime/src/OmniPvdMemoryWriteStreamImpl.cpp
)
SOURCE_GROUP(src FILES ${PVDRUNTIME_SOURCES})

add_library(PVDRuntime SHARED
    ${PVDRUNTIME_HEADERS}
		${PVDRUNTIME_SOURCES}
)

TARGET_INCLUDE_DIRECTORIES(PVDRuntime
  PRIVATE ${PHYSX_ROOT_DIR}/pvdruntime/include
  PRIVATE ${PHYSX_ROOT_DIR}/pvdruntime/src
)

TARGET_COMPILE_DEFINITIONS(PVDRuntime
	PRIVATE ${PVDRUNTME_COMPILE_DEFS}
)

SET_TARGET_PROPERTIES(PVDRuntime PROPERTIES 
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PX_EXE_OUTPUT_DIRECTORY_DEBUG}
    RUNTIME_OUTPUT_DIRECTORY_PROFILE ${PX_EXE_OUTPUT_DIRECTORY_PROFILE}
    RUNTIME_OUTPUT_DIRECTORY_CHECKED ${PX_EXE_OUTPUT_DIRECTORY_CHECKED}
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PX_EXE_OUTPUT_DIRECTORY_RELEASE}

OUTPUT_NAME PVDRuntime
)

TARGET_LINK_LIBRARIES(PVDRuntime 
  PUBLIC ${PVDRUNTIME_PLATFORM_LINKED_LIBS})

IF(PX_GENERATE_SOURCE_DISTRO)
    LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PVDRUNTIME_HEADERS})
    LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PVDRUNTIME_SOURCES})
ENDIF()
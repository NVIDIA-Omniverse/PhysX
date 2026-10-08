## SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
## SPDX-License-Identifier: Apache-2.0

#
# Build Server Test template
#
# Include here after the directories are defined so that the platform specific file can use the variables.

if(DEFINED PVDRUNTIME_ROOT)
  get_filename_component(_PVDRUNTIME_ROOT "${PVDRUNTIME_ROOT}" ABSOLUTE)
else()
  set(_PVDRUNTIME_ROOT "${PHYSX_ROOT_DIR}/pvdruntime")
endif()

include(${_PVDRUNTIME_ROOT}/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/PVDRuntime.cmake)

SET(PVDRUNTIME_HEADERS
  ${_PVDRUNTIME_ROOT}/include/OmniPvdLibraryFunctions.h
  ${_PVDRUNTIME_ROOT}/include/OmniPvdCommands.h
  ${_PVDRUNTIME_ROOT}/include/OmniPvdDefines.h
  ${_PVDRUNTIME_ROOT}/include/OmniPvdReader.h
  ${_PVDRUNTIME_ROOT}/include/OmniPvdWriter.h
  ${_PVDRUNTIME_ROOT}/include/OmniPvdReadStream.h
  ${_PVDRUNTIME_ROOT}/include/OmniPvdWriteStream.h
  ${_PVDRUNTIME_ROOT}/include/OmniPvdFileReadStream.h
  ${_PVDRUNTIME_ROOT}/include/OmniPvdFileWriteStream.h
  ${_PVDRUNTIME_ROOT}/include/OmniPvdMemoryStream.h
  ${_PVDRUNTIME_ROOT}/include/OmniPvdSocketWriteStream.h
  ${_PVDRUNTIME_ROOT}/include/OmniPvdSocketReadStream.h
)
SOURCE_GROUP(include FILES ${PVDRUNTIME_HEADERS})

if(NOT DEFINED PX_ENABLE_INSTALL OR PX_ENABLE_INSTALL)
  install(FILES ${PVDRUNTIME_HEADERS} DESTINATION pvdruntime/include)
endif()

SET(PVDRUNTIME_SOURCES
  ${_PVDRUNTIME_ROOT}/src/OmniPvdDefinesInternal.h
  ${_PVDRUNTIME_ROOT}/src/OmniPvdHelpers.h
  ${_PVDRUNTIME_ROOT}/src/OmniPvdHelpers.cpp
  ${_PVDRUNTIME_ROOT}/src/OmniPvdLibraryFunctionsImpl.cpp
  ${_PVDRUNTIME_ROOT}/src/OmniPvdLog.h
  ${_PVDRUNTIME_ROOT}/src/OmniPvdLog.cpp
  ${_PVDRUNTIME_ROOT}/src/OmniPvdReaderImpl.h
  ${_PVDRUNTIME_ROOT}/src/OmniPvdReaderImpl.cpp
  ${_PVDRUNTIME_ROOT}/src/OmniPvdWriterImpl.h
  ${_PVDRUNTIME_ROOT}/src/OmniPvdWriterImpl.cpp
  ${_PVDRUNTIME_ROOT}/src/OmniPvdFileReadStreamImpl.h
  ${_PVDRUNTIME_ROOT}/src/OmniPvdFileReadStreamImpl.cpp
  ${_PVDRUNTIME_ROOT}/src/OmniPvdFileWriteStreamImpl.h
  ${_PVDRUNTIME_ROOT}/src/OmniPvdFileWriteStreamImpl.cpp
  ${_PVDRUNTIME_ROOT}/src/OmniPvdMemoryStreamImpl.h
  ${_PVDRUNTIME_ROOT}/src/OmniPvdMemoryStreamImpl.cpp
  ${_PVDRUNTIME_ROOT}/src/OmniPvdMemoryReadStreamImpl.h
  ${_PVDRUNTIME_ROOT}/src/OmniPvdMemoryReadStreamImpl.cpp
  ${_PVDRUNTIME_ROOT}/src/OmniPvdMemoryWriteStreamImpl.h
  ${_PVDRUNTIME_ROOT}/src/OmniPvdMemoryWriteStreamImpl.cpp
  ${_PVDRUNTIME_ROOT}/src/OmniPvdSocket.h
  ${_PVDRUNTIME_ROOT}/src/OmniPvdSocket.cpp
  ${_PVDRUNTIME_ROOT}/src/OmniPvdSocketProtocol.h
  ${_PVDRUNTIME_ROOT}/src/OmniPvdSocketWriteStreamImpl.h
  ${_PVDRUNTIME_ROOT}/src/OmniPvdSocketWriteStreamImpl.cpp
  ${_PVDRUNTIME_ROOT}/src/OmniPvdSocketReadStreamImpl.h
  ${_PVDRUNTIME_ROOT}/src/OmniPvdSocketReadStreamImpl.cpp
)
SOURCE_GROUP(src FILES ${PVDRUNTIME_SOURCES})

add_library(PVDRuntime STATIC
    ${PVDRUNTIME_HEADERS}
	${PVDRUNTIME_SOURCES}
)

target_include_directories(PVDRuntime
  PUBLIC
    $<BUILD_INTERFACE:${_PVDRUNTIME_ROOT}/include>
    $<INSTALL_INTERFACE:pvdruntime/include>
  PRIVATE
    ${_PVDRUNTIME_ROOT}/src
)

target_compile_definitions(PVDRuntime
	PRIVATE ${PVDRUNTIME_COMPILE_DEFS}
)

set_target_properties(PVDRuntime PROPERTIES
  POSITION_INDEPENDENT_CODE TRUE
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN TRUE
  OUTPUT_NAME PVDRuntime_static
)

if(DEFINED PVDRUNTIME_COMPILE_PDB_NAME_DEBUG)
  set_target_properties(PVDRuntime PROPERTIES
    COMPILE_PDB_NAME_DEBUG "${PVDRUNTIME_COMPILE_PDB_NAME_DEBUG}"
    COMPILE_PDB_NAME_CHECKED "${PVDRUNTIME_COMPILE_PDB_NAME_CHECKED}"
    COMPILE_PDB_NAME_PROFILE "${PVDRUNTIME_COMPILE_PDB_NAME_PROFILE}"
    COMPILE_PDB_NAME_RELEASE "${PVDRUNTIME_COMPILE_PDB_NAME_RELEASE}"
  )
  if(NOT DEFINED PX_ENABLE_INSTALL OR PX_ENABLE_INSTALL)
    install(FILES "${PX_OUTPUT_LIB_DIR}/$<$<CONFIG:debug>:${PX_ROOT_LIB_DIR}/debug>$<$<CONFIG:release>:${PX_ROOT_LIB_DIR}/release>$<$<CONFIG:checked>:${PX_ROOT_LIB_DIR}/checked>$<$<CONFIG:profile>:${PX_ROOT_LIB_DIR}/profile>/$<$<CONFIG:debug>:${PVDRUNTIME_COMPILE_PDB_NAME_DEBUG}>$<$<CONFIG:checked>:${PVDRUNTIME_COMPILE_PDB_NAME_CHECKED}>$<$<CONFIG:profile>:${PVDRUNTIME_COMPILE_PDB_NAME_PROFILE}>$<$<CONFIG:release>:${PVDRUNTIME_COMPILE_PDB_NAME_RELEASE}>.pdb"
      DESTINATION "$<$<CONFIG:debug>:${PX_ROOT_LIB_DIR}/debug>$<$<CONFIG:release>:${PX_ROOT_LIB_DIR}/release>$<$<CONFIG:checked>:${PX_ROOT_LIB_DIR}/checked>$<$<CONFIG:profile>:${PX_ROOT_LIB_DIR}/profile>"
      OPTIONAL
    )
  endif()
endif()

target_link_libraries(PVDRuntime
  PUBLIC ${PVDRUNTIME_PLATFORM_LINKED_LIBS}
)

IF(PX_GENERATE_SOURCE_DISTRO)
    LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PVDRUNTIME_HEADERS})
    LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PVDRUNTIME_SOURCES})
ENDIF()

unset(_PVDRUNTIME_ROOT)

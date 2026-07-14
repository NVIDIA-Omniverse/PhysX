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
## Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
## Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
## Copyright (c) 2001-2004 NovodeX AG. All rights reserved.


# When included from PhysX SDK build, project() is already called at top level.
# Only declare project and cmake_minimum_required in standalone mode -- calling
# cmake_minimum_required in a subdirectory resets CMP0091 and breaks the parent's
# MSVC runtime library setting (/MT vs /MD).
if(NOT CMAKE_PROJECT_NAME OR CMAKE_PROJECT_NAME STREQUAL "PvdDom")
    cmake_minimum_required(VERSION 3.15)
    cmake_policy(SET CMP0091 NEW)
    project(PvdDom LANGUAGES CXX)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

set(PVDDOM_SOURCES
    src/PvdDomLog.cpp
    src/PvdDom.cpp
    src/PvdDomParser.cpp
    src/PvdDomParserConfig.cpp
    src/PvdDomUtils.cpp
    src/PvdDomChunkAlloc.cpp
)

if(TARGET PVDRuntime)
    # -------------------------------------------------------------------------
    # PhysX SDK build mode: PVDRuntime target already exists (shared library).
    # Link against it instead of compiling sources in.
    # -------------------------------------------------------------------------
    add_library(PvdDom STATIC ${PVDDOM_SOURCES})

    # The archive gets linked into shared objects (e.g. libomni.physx.pvd.plugin.so),
    # so the objects must be position-independent on Linux/clang/gcc.
    set_target_properties(PvdDom PROPERTIES POSITION_INDEPENDENT_CODE ON)

    # Match the PhysX SDK's MSVC runtime library.  CMP0091/MSVC_RUNTIME_LIBRARY
    # is unreliable across subdirectory boundaries with CMake 3.25 + ninja, so
    # strip the default /MD flags and replace with the correct CRT choice.
    if(MSVC)
        foreach(_cfg CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE CMAKE_CXX_FLAGS_CHECKED CMAKE_CXX_FLAGS_PROFILE CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO)
            string(REGEX REPLACE "/MD[d]?" "" ${_cfg} "${${_cfg}}")
        endforeach()
        if(NV_USE_STATIC_WINCRT)
            target_compile_options(PvdDom PRIVATE
                $<$<CONFIG:Debug>:/MTd>
                $<$<NOT:$<CONFIG:Debug>>:/MT>
            )
        else()
            target_compile_options(PvdDom PRIVATE
                $<$<CONFIG:Debug>:/MDd>
                $<$<NOT:$<CONFIG:Debug>>:/MD>
            )
        endif()
    endif()

    target_include_directories(PvdDom
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${PHYSX_ROOT_DIR}/pvdruntime/include
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src
            ${PHYSX_ROOT_DIR}/pvdruntime/src
    )

    target_link_libraries(PvdDom PUBLIC PVDRuntime)

else()
    # -------------------------------------------------------------------------
    # Standalone mode (e.g. PVD3): compile pvdruntime sources directly into
    # pvddom. No DLL loading, no external PVDRuntime dependency.
    # -------------------------------------------------------------------------
    if(NOT DEFINED PVDRUNTIME_ROOT)
        message(FATAL_ERROR "PVDRUNTIME_ROOT must be set to the pvdruntime directory (standalone mode)")
    endif()

    set(PVDRUNTIME_SOURCES
        ${PVDRUNTIME_ROOT}/src/OmniPvdHelpers.cpp
        ${PVDRUNTIME_ROOT}/src/OmniPvdLibraryFunctionsImpl.cpp
        ${PVDRUNTIME_ROOT}/src/OmniPvdLog.cpp
        ${PVDRUNTIME_ROOT}/src/OmniPvdReaderImpl.cpp
        ${PVDRUNTIME_ROOT}/src/OmniPvdWriterImpl.cpp
        ${PVDRUNTIME_ROOT}/src/OmniPvdFileReadStreamImpl.cpp
        ${PVDRUNTIME_ROOT}/src/OmniPvdFileWriteStreamImpl.cpp
        ${PVDRUNTIME_ROOT}/src/OmniPvdMemoryStreamImpl.cpp
        ${PVDRUNTIME_ROOT}/src/OmniPvdMemoryReadStreamImpl.cpp
        ${PVDRUNTIME_ROOT}/src/OmniPvdMemoryWriteStreamImpl.cpp
    )

    add_library(PvdDom STATIC ${PVDDOM_SOURCES} ${PVDRUNTIME_SOURCES})

    # May be linked into shared objects downstream; keep objects position-independent.
    set_target_properties(PvdDom PROPERTIES POSITION_INDEPENDENT_CODE ON)

    target_include_directories(PvdDom
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${PVDRUNTIME_ROOT}/include
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src
            ${PVDRUNTIME_ROOT}/src
    )
endif()

# Suppress C4005 (OMNI_PVD_EXPORT macro redefinition) on MSVC
if(MSVC)
    target_compile_options(PvdDom PRIVATE /wd4005)
endif()

# Tests (optional)
option(PVDDOM_BUILD_TESTS "Build pvddom tests" OFF)
if(PVDDOM_BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()

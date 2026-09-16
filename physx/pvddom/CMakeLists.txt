## Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
## Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
## SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
## SPDX-License-Identifier: Apache-2.0



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
    src/PvdDomQuery.cpp
    src/PvdDomUtils.cpp
    src/PvdDomChunkAlloc.cpp
)

if(NOT TARGET PVDRuntime)
    # Standalone mode (e.g. PVD3): create the same static runtime target used by
    # the PhysX SDK build, then link PvdDom to it below.
    if(NOT DEFINED PVDRUNTIME_ROOT)
        message(FATAL_ERROR "PVDRUNTIME_ROOT must be set to the pvdruntime directory (standalone mode)")
    endif()

    if(WIN32)
        set(_PVDDOM_RUNTIME_PLATFORM windows)
    elseif(UNIX)
        set(_PVDDOM_RUNTIME_PLATFORM linux)
    else()
        message(FATAL_ERROR "Standalone PVDRuntime is unsupported on this platform")
    endif()

    function(_pvddom_add_standalone_runtime runtime_root platform)
        get_filename_component(PVDRUNTIME_ROOT "${runtime_root}" ABSOLUTE)
        get_filename_component(PHYSX_ROOT_DIR "${PVDRUNTIME_ROOT}" DIRECTORY)
        set(PROJECT_CMAKE_FILES_DIR compiler/cmake)
        set(TARGET_BUILD_PLATFORM "${platform}")
        set(PX_ENABLE_INSTALL OFF)
        include("${PVDRUNTIME_ROOT}/compiler/cmake/PVDRuntime.cmake")

        # Keep the standalone runtime on the same CRT as PvdDom. The integrated
        # PhysX path configures this on the existing target before reaching here.
        if(MSVC)
            if(NV_USE_STATIC_WINCRT)
                target_compile_options(PVDRuntime PRIVATE
                    $<$<CONFIG:Debug>:/MTd>
                    $<$<NOT:$<CONFIG:Debug>>:/MT>
                )
            else()
                target_compile_options(PVDRuntime PRIVATE
                    $<$<CONFIG:Debug>:/MDd>
                    $<$<NOT:$<CONFIG:Debug>>:/MD>
                )
            endif()
        endif()
    endfunction()

    _pvddom_add_standalone_runtime("${PVDRUNTIME_ROOT}" "${_PVDDOM_RUNTIME_PLATFORM}")
endif()

add_library(PvdDom STATIC ${PVDDOM_SOURCES})

# The archive gets linked into shared objects (e.g. libomni.physx.pvd.plugin.so),
# so the objects must be position-independent on Linux/clang/gcc.
set_target_properties(PvdDom PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Match the PhysX SDK's MSVC runtime library. CMP0091/MSVC_RUNTIME_LIBRARY is
# unreliable across subdirectory boundaries with CMake 3.25 + Ninja, so strip
# the default /MD flags and replace them with the host project's CRT choice.
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
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(PvdDom PUBLIC PVDRuntime)

# Tests (optional)
option(PVDDOM_BUILD_TESTS "Build pvddom tests" OFF)
if(PVDDOM_BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()

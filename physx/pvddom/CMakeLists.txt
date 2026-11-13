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

option(PVDDOM_BUILD_INSPECTION "Build bounded OVD inspection APIs" OFF)

set(PVDDOM_SOURCES
    src/PvdDomLog.cpp
    src/PvdDom.cpp
    src/PvdDomParser.cpp
    src/PvdDomParserConfig.cpp
    src/PvdDomQuery.cpp
    src/PvdDomUtils.cpp
    src/PvdDomChunkAlloc.cpp
)
if(PVDDOM_BUILD_INSPECTION)
    list(APPEND PVDDOM_SOURCES
        src/PvdDomDocument.cpp
        src/PvdDomInspection.cpp
        src/PvdDomEvidence.cpp
    )
endif()

if(NOT TARGET PVDRuntime)
    # Standalone mode (e.g. PVD3): create the same static runtime target used by
    # the PhysX SDK build, then link PvdDom to it below.
    if(NOT DEFINED PVDRUNTIME_ROOT)
        message(FATAL_ERROR "PVDRUNTIME_ROOT must be set to the pvdruntime directory (standalone mode)")
    endif()
    file(TO_CMAKE_PATH "${PVDRUNTIME_ROOT}" PVDRUNTIME_ROOT)
    get_filename_component(PVDRUNTIME_ROOT "${PVDRUNTIME_ROOT}" ABSOLUTE)

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

if(MSVC)
    # This directory builds the ordinary PvdDom core without C++ exceptions.
    string(REGEX REPLACE "(^|[ \\t])/EHsc([ \\t]|$)" " " CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
endif()
add_library(PvdDom STATIC ${PVDDOM_SOURCES})

if(PVDDOM_BUILD_INSPECTION)
    target_compile_definitions(PvdDom PRIVATE PVDDOM_INSPECTION_ENABLED)
    if(MSVC)
        target_compile_options(PvdDom PRIVATE /EHsc)
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(PvdDom PRIVATE -fexceptions)
    endif()
elseif(MSVC)
    target_compile_definitions(PvdDom PRIVATE _HAS_EXCEPTIONS=0)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(PvdDom PRIVATE -fno-exceptions)
endif()

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
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/pvddom>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(PvdDom PUBLIC PVDRuntime)
target_compile_features(PvdDom PUBLIC cxx_std_14)

option(PVDDOM_BUILD_TOOLS "Build the SDK OVD inspection CLI" OFF)
if(PVDDOM_BUILD_TOOLS AND NOT PVDDOM_BUILD_INSPECTION)
    message(FATAL_ERROR "PVDDOM_BUILD_TOOLS requires PVDDOM_BUILD_INSPECTION=ON")
endif()
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/../include/foundation/PxPhysicsVersion.h" _version_lines
    REGEX "#define PX_PHYSICS_VERSION_(MAJOR|MINOR|BUGFIX)")
foreach(_line IN LISTS _version_lines)
    if(_line MATCHES "#define PX_PHYSICS_VERSION_(MAJOR|MINOR|BUGFIX)[ \t]+([0-9]+)")
        set(_version_${CMAKE_MATCH_1} "${CMAKE_MATCH_2}")
    endif()
endforeach()
set(_pvddom_sdk_version "${_version_MAJOR}.${_version_MINOR}.${_version_BUGFIX}")
if(PVDDOM_BUILD_TOOLS)
    add_executable(ovd_inspect tools/ovd_inspect.cpp)
    if(MSVC)
        target_compile_options(ovd_inspect PRIVATE /EHsc)
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(ovd_inspect PRIVATE -fexceptions)
    endif()
    set_target_properties(ovd_inspect PROPERTIES OUTPUT_NAME ovd-inspect)
    target_link_libraries(ovd_inspect PRIVATE PvdDom)
    if(MSVC AND NV_USE_STATIC_WINCRT)
        target_compile_options(ovd_inspect PRIVATE $<$<CONFIG:Debug>:/MTd> $<$<NOT:$<CONFIG:Debug>>:/MT>)
    endif()
    target_include_directories(ovd_inspect PRIVATE src)
    target_compile_definitions(ovd_inspect PRIVATE
        PVDDOM_SDK_VERSION="${_pvddom_sdk_version}"
        PVDDOM_BUILD_INFO="${CMAKE_CXX_COMPILER_ID}-${CMAKE_CXX_COMPILER_VERSION}-$<CONFIG>")
    install(TARGETS ovd_inspect RUNTIME DESTINATION bin)
endif()

# Standalone exports carry both archives and all public headers.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    include(CMakePackageConfigHelpers)
    configure_package_config_file(cmake/PvdDomConfig.cmake
        "${CMAKE_CURRENT_BINARY_DIR}/PvdDomConfig.cmake"
        INSTALL_DESTINATION lib/cmake/PvdDom)
    write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/PvdDomConfigVersion.cmake"
        VERSION "${_pvddom_sdk_version}" COMPATIBILITY SameMajorVersion)
    install(TARGETS PvdDom EXPORT PvdDomTargets ARCHIVE DESTINATION lib/$<CONFIG>)
    install(TARGETS PVDRuntime EXPORT PvdDomRuntimeTargets ARCHIVE DESTINATION lib/$<CONFIG>)
    install(FILES
        include/PvdDom.h
        include/PvdDomLog.h
        include/PvdDomParser.h
        include/PvdDomQuery.h
        include/PvdDomUtils.h
        DESTINATION include/pvddom)
    if(PVDDOM_BUILD_INSPECTION)
        install(FILES
            include/PvdDomDocument.h
            include/PvdDomInspection.h
            DESTINATION include/pvddom)
    endif()
    install(DIRECTORY "${PVDRUNTIME_ROOT}/include/" DESTINATION pvdruntime/include FILES_MATCHING PATTERN "*.h")
    install(EXPORT PvdDomTargets NAMESPACE PhysX:: DESTINATION lib/cmake/PvdDom)
    install(EXPORT PvdDomRuntimeTargets NAMESPACE PvdDom:: DESTINATION lib/cmake/PvdDom)
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/PvdDomConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/PvdDomConfigVersion.cmake" DESTINATION lib/cmake/PvdDom)
    install(DIRECTORY ../skills/ DESTINATION share/physx/skills)
    install(FILES ../SKILLS.md DESTINATION share/physx)
endif()

# Tests (optional)
option(PVDDOM_BUILD_TESTS "Build pvddom tests" OFF)
if(PVDDOM_BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()

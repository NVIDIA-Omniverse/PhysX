# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# ovphysx Internal Sidecar Library
# Separates omni.physx-dependent functionality into a peer DSO that the main
# library loads on demand. Neither library links OpenUSD.
#
# @implements REQ-PACKAGING-USDFREE-001
# @covers AC-2 AC-3

cmake_minimum_required(VERSION 3.16)

add_library(ovphysx_internal SHARED
    ovphysxInternal.cpp
    ovphysxInternalBootstrap.cpp
    ovphysxInternalInterop.cpp
    ovphysxInternalObjectChange.cpp
)

# The sidecar is a peer DSO following the ovrtx pattern: the owner uses
# CARB_STATIC_BINARY_GLOBALS, peers use CARB_GLOBALS plus setter injection.
# It does not define CARB_STATIC_PLUGIN_BUILD and does not link carb.static.
# The main library injects the framework and OmniCore built-ins at load.
if(NOT DEFINED CARB_STATIC_INCLUDE_DIR OR CARB_STATIC_INCLUDE_DIR STREQUAL "")
    message(FATAL_ERROR "ovphysx_internal requires CARB_STATIC_INCLUDE_DIR for Carbonite headers")
endif()
target_include_directories(ovphysx_internal BEFORE PRIVATE ${CARB_STATIC_INCLUDE_DIR})
target_compile_definitions(ovphysx_internal PRIVATE
    carb_ErrorApi=carb_ErrorApi_latest
    carb_Framework=carb_Framework_latest
    carb_filesystem_IFileSystem=carb_filesystem_IFileSystem_latest
    carb_dictionary_IDictionary=carb_dictionary_IDictionary_latest
)
if(WIN32)
    target_link_libraries(ovphysx_internal PRIVATE shlwapi pathcch winmm)
    target_compile_options(ovphysx_internal PRIVATE /wd4211)
endif()

# Link the main library. The sidecar reuses its runtime and links no USD.
target_link_libraries(ovphysx_internal PRIVATE ovphysx)

target_include_directories(ovphysx_internal PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/include
)
# Third-party headers as SYSTEM to suppress warnings from code outside this project.
target_include_directories(ovphysx_internal SYSTEM PRIVATE
    ${CARB_SDK_INCLUDE_DIR}
    ${OMNI_PHYSICS_DIR}/include
    ${LOCAL_TARGET_DEPS}/gsl/include
    # PhysX SDK headers. The kinematic-update entry point calls
    # PxArticulationReducedCoordinate::updateKinematic directly. Other sidecar
    # entry points only use omni::physx interfaces, not PhysX SDK types.
    ${OVRUNTIME_DIR}/_build/target-deps/physx/include
    # ovstage headers (ovx/, ovstage/ovstage_api/, dlpack/) pulled in by
    # omni/physx/IOvxPhysicsRead.h for the physics output read entry points.
    # Use the exact provider resolved by OvstageDependency.cmake.
    ${OVSTAGE_INCLUDE_DIRS}
)

ovphysx_apply_compiler_defaults(ovphysx_internal
    GCC_SUPPRESSIONS
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-enum-compare>
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-class-memaccess>
)

# Export the sidecar entry points from this DLL.
target_compile_definitions(ovphysx_internal PRIVATE
    OVPHYSX_INTERNAL_API_EXPORTS
)

# Windows-specific defines to prevent macro conflicts
if(WIN32)
    target_compile_definitions(ovphysx_internal PRIVATE
        NOMINMAX            # Prevent Windows.h min/max macros from breaking STL
        WIN32_LEAN_AND_MEAN # Reduce Windows.h bloat
    )
endif()

if(WIN32)
    set_target_properties(ovphysx_internal PROPERTIES
        OUTPUT_NAME "ovphysx_internal"
        RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_DIR}"
    )
else()
    set_target_properties(ovphysx_internal PROPERTIES
        OUTPUT_NAME "ovphysx_internal"
        LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_DIR}"
        VERSION ${OVPHYSX_VERSION_NUMERIC}
        SOVERSION ${OVPHYSX_VERSION_MAJOR}
    )
endif()

# On Linux, keep the sidecar relocatable beside the main library and the
# runtime plugins at both build time and install time.
if(NOT WIN32)
    set_target_properties(ovphysx_internal PROPERTIES
        BUILD_RPATH "\$ORIGIN"
        INSTALL_RPATH "\$ORIGIN:\$ORIGIN/../plugins"
        BUILD_RPATH_USE_ORIGIN TRUE
        SKIP_BUILD_RPATH FALSE
        LINK_FLAGS "-Wl,--disable-new-dtags"
    )
endif()

message(STATUS "Internal sidecar will be built as separate shared library")

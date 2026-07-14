# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

cmake_minimum_required(VERSION 3.22)

project(physics-umbrella)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
      ${CMAKE_CURRENT_SOURCE_DIR}/cmake        # umbrella-specific (Options.cmake)
      ${CMAKE_CURRENT_SOURCE_DIR}/../cmake     # shared omni/cmake modules (NvidiaBuildOptions, CompilerDefaults, etc.)
)

set(UMBRELLA_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# When included as a subdirectory (e.g. from ovexts), the parent project sets
# UMBRELLA_TARGET_DEPS, CARB_SDK_DIR, and other variables before add_subdirectory().
# Skip dependency discovery and shared module includes that the parent already handled.
set(_UMBRELLA_STANDALONE TRUE)
if(NOT CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(_UMBRELLA_STANDALONE FALSE)
endif()

if(_UMBRELLA_STANDALONE)
    # Platform detection (can be overridden via -DTARGET_BUILD_PLATFORM=...)
    if(NOT DEFINED TARGET_BUILD_PLATFORM)
        if(WIN32)
            set(TARGET_BUILD_PLATFORM "windows")
        elseif(UNIX AND NOT APPLE)
            set(TARGET_BUILD_PLATFORM "linux")
        elseif(APPLE)
            set(TARGET_BUILD_PLATFORM "mac")
        endif()
    endif()

    # Required by NvidiaBuildOptions for output directory structure
    set(PX_OUTPUT_LIB_DIR "${CMAKE_BINARY_DIR}" CACHE PATH "Base lib output directory")
    set(PX_OUTPUT_BIN_DIR "${CMAKE_BINARY_DIR}" CACHE PATH "Base bin output directory")
endif()

# Find Carbonite SDK from kit-sdk dependencies (pulled via packman)
if(NOT DEFINED UMBRELLA_TARGET_DEPS)
    set(UMBRELLA_TARGET_DEPS "${CMAKE_CURRENT_SOURCE_DIR}/_build/target-deps")
endif()
if(NOT DEFINED CARB_SDK_DIR)
    if(EXISTS "${UMBRELLA_TARGET_DEPS}/carb_sdk_plugins/include")
        set(CARB_SDK_DIR "${UMBRELLA_TARGET_DEPS}/carb_sdk_plugins")
        message(STATUS "Found Carbonite SDK: ${CARB_SDK_DIR}")
    else()
        message(FATAL_ERROR "Carbonite SDK not found at ${UMBRELLA_TARGET_DEPS}/carb_sdk_plugins.")
    endif()
endif()

include(Options)              # umbrella-specific options (UMBRELLA_ENABLE_TESTS, etc.)
if(_UMBRELLA_STANDALONE)
    include(NvidiaBuildOptions)   # shared: CRT, output dirs, platform naming
    include(CarbPlatform)         # shared: CARB_PLATFORM_DIR for Carbonite SDK lib path
endif()
# CompilerDefaults must be included unconditionally — it sets directory-scoped
# flags (-fvisibility=hidden, -Werror, etc.) that must apply to umbrella targets
# even when built as a subproject.  Without this, the subproject build leaks
# internal symbols and misses warning/optimization flags.
include(CompilerDefaults)         # shared: compiler warning flags, optimizations

# C++ standard and position-independent code
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Configuration types — keep Debug/Release only (Checked/Profile are PhysX-specific
# and require additional linker flag variables that CMake does not define by default)
if(CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "" FORCE)
endif()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

message(STATUS "Physics Umbrella root dir: ${CMAKE_CURRENT_SOURCE_DIR}")

if(UMBRELLA_ENABLE_TESTS)
    enable_testing()
endif()

add_subdirectory(source)
add_subdirectory(bindings)
if(_UMBRELLA_STANDALONE)
    add_subdirectory(tests)
endif()

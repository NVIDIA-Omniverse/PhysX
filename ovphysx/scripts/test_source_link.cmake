# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# ovphysx source-link sample test.
# Configures, builds, and runs the hello_world_source_link sample, which uses
# add_subdirectory() to build against the ovphysx source tree.
#
# The build verifies that add_subdirectory() integration works (configure,
# compile, link). At runtime, OVPHYSX_LIB points discovery at the installed
# SDK layout, which has the complete flattened plugin tree.
#
# Usage: cmake [-DBUILD_TYPE=Release] [-DOVPHYSX_DEV_PHYSX=ON|OFF]
#              [-DSOURCE_LINK_CONTEXT_FILE=<generated-context>]
#              [-DSOURCE_LINK_GENERATOR=<manual-generator>]
#              -P scripts/test_source_link.cmake

cmake_minimum_required(VERSION 3.16)

get_filename_component(SCRIPT_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
include("${SCRIPT_DIR}/../cmake/SourceLinkContext.cmake")

# CTest/custom-target processes may start without the Packman environment.
# Restore it before build_common.cmake performs any find_program() calls.
if(DEFINED SOURCE_LINK_CONTEXT_FILE AND NOT SOURCE_LINK_CONTEXT_FILE STREQUAL "")
    if(NOT IS_ABSOLUTE "${SOURCE_LINK_CONTEXT_FILE}")
        get_filename_component(SOURCE_LINK_CONTEXT_FILE
            "${SOURCE_LINK_CONTEXT_FILE}" ABSOLUTE
            BASE_DIR "${CMAKE_CURRENT_BINARY_DIR}")
    endif()
    if(NOT EXISTS "${SOURCE_LINK_CONTEXT_FILE}")
        message(FATAL_ERROR
            "Source-link context file not found: ${SOURCE_LINK_CONTEXT_FILE}")
    endif()
    include("${SOURCE_LINK_CONTEXT_FILE}")
    ovphysx_apply_source_link_environment()
endif()

include("${SCRIPT_DIR}/crossplatform_helpers.cmake")
include("${SCRIPT_DIR}/build_common.cmake")

set(SAMPLE_DIR "${PROJECT_ROOT}/tests/c_samples/hello_world_source_link")
set(SAMPLE_BUILD_DIR "${PROJECT_ROOT}/_build/sample_tests/c_samples/hello_world_source_link")
set(SAMPLE_NAME "hello_world_source_link")
set(INSTALL_PATH "${PROJECT_ROOT}/_install")
set(INSTALL_LIB_PATH "${INSTALL_PATH}/${INSTALL_RUNTIME_SUBDIR}/${SHARED_LIB_PREFIX}ovphysx${SHARED_LIB_SUFFIX}")
if(NOT DEFINED OVPHYSX_DEV_PHYSX)
    set(OVPHYSX_DEV_PHYSX OFF)
endif()

message(STATUS "")
message(STATUS "=== Source-Link Sample Test (testing add_subdirectory workflow) ===")
message(STATUS "Sample: ${SAMPLE_DIR}")
message(STATUS "Build dir: ${SAMPLE_BUILD_DIR}")
message(STATUS "Config: ${BUILD_TYPE}")
message(STATUS "Development PhysX: ${OVPHYSX_DEV_PHYSX}")
message(STATUS "Context: ${SOURCE_LINK_CONTEXT_FILE}")

# The installed SDK provides the runtime plugins.
if(NOT EXISTS "${INSTALL_PATH}/plugins")
    message(FATAL_ERROR "Installed SDK not found at ${INSTALL_PATH}.\n"
                        "Run: cmake -P scripts/install.cmake")
endif()
if(NOT EXISTS "${INSTALL_LIB_PATH}")
    message(FATAL_ERROR "Installed ovphysx library not found at ${INSTALL_LIB_PATH}.\n"
                        "Run: cmake -P scripts/install.cmake")
endif()

set(_SOURCE_LINK_OVSTAGE_ARGS)
if(DEFINED OVSTAGE_DIR AND EXISTS "${OVSTAGE_DIR}")
    list(APPEND _SOURCE_LINK_OVSTAGE_ARGS "-DOVSTAGE_DIR=${OVSTAGE_DIR}")
elseif(EXISTS "${PROJECT_ROOT}/_build/CMakeCache.txt")
    file(STRINGS "${PROJECT_ROOT}/_build/CMakeCache.txt" _OVSTAGE_CACHE_LINE REGEX "^OVSTAGE_DIR(:[^=]*)?=")
    if(_OVSTAGE_CACHE_LINE)
        list(GET _OVSTAGE_CACHE_LINE 0 _OVSTAGE_CACHE_LINE_FIRST)
        string(REGEX REPLACE "^[^=]+=" "" _OVSTAGE_CACHE_DIR "${_OVSTAGE_CACHE_LINE_FIRST}")
        if(EXISTS "${_OVSTAGE_CACHE_DIR}")
            list(APPEND _SOURCE_LINK_OVSTAGE_ARGS "-DOVSTAGE_DIR=${_OVSTAGE_CACHE_DIR}")
        endif()
    endif()
endif()

file(REMOVE_RECURSE "${SAMPLE_BUILD_DIR}")
file(MAKE_DIRECTORY "${SAMPLE_BUILD_DIR}")

# Toolchain values that may contain CMake list separators are transported in an
# initial-cache file instead of command-line -D arguments. Generator selection
# remains separate so unsupported generators never receive -A or -T.
set(_SOURCE_LINK_INITIAL_CACHE
    "${SAMPLE_BUILD_DIR}/source_link_initial_cache.cmake")
ovphysx_write_source_link_initial_cache("${_SOURCE_LINK_INITIAL_CACHE}")
set(SOURCE_LINK_CUDA_TOOLSET_DIR
    "${PROJECT_ROOT}/ovruntime/_build/target-deps/cuda")
ovphysx_get_source_link_generator_args(
    _SAMPLE_GENERATOR_ARGS _SAMPLE_GENERATOR
    "${_SOURCE_LINK_INITIAL_CACHE}")
message(STATUS "Generator: ${_SAMPLE_GENERATOR}")

# The main build has already fetched the deps, so auto-fetch is skipped.
message(STATUS "Configuring ${SAMPLE_NAME}...")
execute_process(
    COMMAND ${CMAKE_COMMAND}
        ${_SAMPLE_GENERATOR_ARGS}
        -S "${SAMPLE_DIR}"
        -B "${SAMPLE_BUILD_DIR}"
        "-DCMAKE_BUILD_TYPE=${BUILD_TYPE}"
        -DOVPHYSX_FETCH_DEPS=OFF
        "-DOVPHYSX_DEV_PHYSX=${OVPHYSX_DEV_PHYSX}"
        ${_SOURCE_LINK_OVSTAGE_ARGS}
    WORKING_DIRECTORY "${PROJECT_ROOT}"
    RESULT_VARIABLE CONFIG_RESULT
)
if(NOT CONFIG_RESULT STREQUAL "0")
    message(FATAL_ERROR "${SAMPLE_NAME} configuration failed (exit code: ${CONFIG_RESULT})")
endif()

# Build the default target, which includes ovphysx, the ovruntime plugins, and the sample.
# Parallelism is bounded because this rebuilds the full ovruntime/PhysX stack from source,
# and an unbounded `--parallel` (equivalent to make -j) exhausts RAM.
ovphysx_compute_build_jobs(_SAMPLE_JOBS)
message(STATUS "Building ${SAMPLE_NAME} (parallel ${_SAMPLE_JOBS})...")
execute_process(
    COMMAND ${CMAKE_COMMAND} --build "${SAMPLE_BUILD_DIR}"
        --parallel ${_SAMPLE_JOBS} --config ${BUILD_TYPE}
    WORKING_DIRECTORY "${PROJECT_ROOT}"
    RESULT_VARIABLE BUILD_RESULT
)
if(NOT BUILD_RESULT STREQUAL "0")
    message(FATAL_ERROR "${SAMPLE_NAME} build failed (exit code: ${BUILD_RESULT})")
endif()

file(GLOB_RECURSE SAMPLE_EXECUTABLE_LIST "${SAMPLE_BUILD_DIR}/*${SAMPLE_NAME}${EXE_SUFFIX}")
if(NOT SAMPLE_EXECUTABLE_LIST)
    message(FATAL_ERROR "${SAMPLE_NAME} executable not found in ${SAMPLE_BUILD_DIR}")
endif()
list(GET SAMPLE_EXECUTABLE_LIST 0 SAMPLE_EXECUTABLE)
message(STATUS "Found executable: ${SAMPLE_EXECUTABLE}")

# Run using the installed SDK layout for runtime config/schema/plugin discovery.
# The executable still finds its linked build-tree libraries via RPATH/DLL copy.
# OVPHYSX_LIB only anchors ovphysx runtime discovery at the installed SDK.
if(WIN32)
    # ovstage is not installed, so the external runtime is put on PATH.
    ovphysx_resolve_ovstage_paths()
    get_filename_component(SAMPLE_EXECUTABLE_DIR "${SAMPLE_EXECUTABLE}" DIRECTORY)
    set(_WINDOWS_PATH_SEGMENTS
        "${SAMPLE_EXECUTABLE_DIR}"
        "${INSTALL_PATH}/${INSTALL_RUNTIME_SUBDIR}"
        "${INSTALL_PATH}/plugins"
        "${INSTALL_PATH}/plugins/bin/deps"
        "${OVPHYSX_OVSTAGE_RUNTIME_DIR}"
        # ovstage.dll statically imports the USD monolith, which lives in
        # OVStage's plugins/ dir and which the SDK does not ship
        # (REQ-PACKAGING-USDFREE-001 AC-4). Linux resolves it via ovstage's
        # RUNPATH ($ORIGIN/plugins). Windows needs it on PATH.
        "${OVPHYSX_OVSTAGE_RUNTIME_DIR}/plugins"
        "$ENV{SystemRoot}\\System32"
        "$ENV{SystemRoot}"
    )
    file(GLOB_RECURSE _SOURCE_LINK_DLLS "${SAMPLE_BUILD_DIR}/*.dll")
    foreach(_SOURCE_LINK_DLL ${_SOURCE_LINK_DLLS})
        get_filename_component(_SOURCE_LINK_DLL_DIR "${_SOURCE_LINK_DLL}" DIRECTORY)
        list(APPEND _WINDOWS_PATH_SEGMENTS "${_SOURCE_LINK_DLL_DIR}")
    endforeach()
    if(DEFINED ENV{CUDA_PATH})
        list(APPEND _WINDOWS_PATH_SEGMENTS "$ENV{CUDA_PATH}\\bin")
    endif()
    list(REMOVE_DUPLICATES _WINDOWS_PATH_SEGMENTS)
    list(JOIN _WINDOWS_PATH_SEGMENTS ";" _WINDOWS_PATH_VALUE)
    string(REPLACE ";" "\\;" _WINDOWS_PATH_VALUE_ESCAPED "${_WINDOWS_PATH_VALUE}")
    set(_ENV_OVERRIDES
        "PATH=${_WINDOWS_PATH_VALUE_ESCAPED}"
        "OVPHYSX_LIB=${INSTALL_LIB_PATH}"
    )
else()
    # LD_LIBRARY_PATH is cleared to verify the executable finds its libraries via RPATH alone.
    set(_ENV_OVERRIDES "LD_LIBRARY_PATH=" "OVPHYSX_LIB=${INSTALL_LIB_PATH}")
endif()

message(STATUS "Running ${SAMPLE_NAME} (plugins from installed SDK)...")
execute_process(
    COMMAND ${CMAKE_COMMAND} -E env ${_ENV_OVERRIDES} "${SAMPLE_EXECUTABLE}"
    WORKING_DIRECTORY ${PROJECT_ROOT}
    TIMEOUT 60
    RESULT_VARIABLE RUN_RESULT
)

if(NOT RUN_RESULT STREQUAL "0")
    message(FATAL_ERROR "${SAMPLE_NAME} failed (exit code: ${RUN_RESULT})")
endif()

message(STATUS "")
message(STATUS "[PASS] ${SAMPLE_NAME} passed")
message(STATUS "")
message(STATUS "Source-link sample test: PASSED")

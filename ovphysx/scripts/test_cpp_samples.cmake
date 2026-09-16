# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# ovphysx C++ sample tests.
# Runs all C/C++ sample applications against the installed SDK (_install/).
# Usage: cmake -P scripts/test_cpp_samples.cmake
#
# The samples must test the installed SDK, not the build tree.

cmake_minimum_required(VERSION 3.16)

get_filename_component(SCRIPT_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
include("${SCRIPT_DIR}/crossplatform_helpers.cmake")
include("${SCRIPT_DIR}/build_common.cmake")

set(INSTALL_PATH "${PROJECT_ROOT}/_install")

# ovstage is not installed. The samples build and run against the external one.
ovphysx_resolve_ovstage_paths()

message(STATUS "")
message(STATUS "=== C++ Sample Tests (testing installed SDK) ===")
message(STATUS "Install path: ${INSTALL_PATH}")

# In CI artifact-consumer jobs, _install/ may be pre-populated from an SDK archive
# with no local _build/ tree. This flag bypasses the build-tree staleness checks.
if(NOT DEFINED SKIP_STALENESS_CHECK)
    set(SKIP_STALENESS_CHECK OFF)
endif()

# Split-pipeline modes. SAMPLES_BUILD_ONLY compiles every sample but skips
# running them (build runners without a GPU). SAMPLES_RUN_ONLY runs the
# executables prebuilt into _build/sample_tests by an earlier BUILD_ONLY pass
# (GPU test runners without a compiler). The default is build and run.
if(NOT DEFINED SAMPLES_BUILD_ONLY)
    set(SAMPLES_BUILD_ONLY OFF)
endif()
if(NOT DEFINED SAMPLES_RUN_ONLY)
    set(SAMPLES_RUN_ONLY OFF)
endif()
if(SAMPLES_BUILD_ONLY AND SAMPLES_RUN_ONLY)
    message(FATAL_ERROR "SAMPLES_BUILD_ONLY and SAMPLES_RUN_ONLY are mutually exclusive")
endif()

# ============================================================================
# Installed SDK staleness check
# ============================================================================
# C++ samples use find_package() to locate the SDK, which requires:
#   1. CMake config files (_install/lib/cmake/ovphysx/ovphysxConfig.cmake)
#   2. Up-to-date installed libraries
#
# Staleness is detected by comparing the timestamps of the built and installed
# SDK library.
# ============================================================================

set(CONFIG_FILE "${PROJECT_ROOT}/_install/lib/cmake/ovphysx/ovphysxConfig.cmake")
set(NEEDS_INSTALL FALSE)

if(NOT SKIP_STALENESS_CHECK)
    if(NOT EXISTS "${CONFIG_FILE}")
        set(NEEDS_INSTALL TRUE)
        message(STATUS "CMake config not found, full install needed")
    else()
        set(LIB_NAME "${SHARED_LIB_PREFIX}ovphysx${SHARED_LIB_SUFFIX}")
        set(BUILT_LIB "${PROJECT_ROOT}/_build/${PLATFORM_NAME}/${BUILD_TYPE_LOWER}/${LIB_NAME}")
        set(INSTALLED_LIB "${PROJECT_ROOT}/_install/${INSTALL_RUNTIME_SUBDIR}/${LIB_NAME}")
        
        if(EXISTS "${BUILT_LIB}")
            if(NOT EXISTS "${INSTALLED_LIB}")
                set(NEEDS_INSTALL TRUE)
                message(STATUS "Installed SDK library not found, full install needed")
            else()
                file(TIMESTAMP "${BUILT_LIB}" BUILT_TIME "%s")
                file(TIMESTAMP "${INSTALLED_LIB}" INSTALLED_TIME "%s")
                if(BUILT_TIME GREATER INSTALLED_TIME)
                    set(NEEDS_INSTALL TRUE)
                    message(STATUS "Installed SDK is stale (built: ${BUILT_TIME}, installed: ${INSTALLED_TIME}), full install needed")
                endif()
            endif()
        else()
            message(FATAL_ERROR "Built SDK library not found at: ${BUILT_LIB}\nRun: cmake -P scripts/build.cmake")
        endif()
    endif()
else()
    if(NOT EXISTS "${CONFIG_FILE}")
        message(FATAL_ERROR "SKIP_STALENESS_CHECK is ON but ${CONFIG_FILE} not found.\n"
                            "Ensure _install/ is populated (e.g. extract SDK archive first).")
    endif()
    message(STATUS "Staleness check skipped (SKIP_STALENESS_CHECK=ON)")
endif()

if(NEEDS_INSTALL)
    message(FATAL_ERROR
        "Installed SDK is missing or stale.\n"
        "Run: cmake -P scripts/install.cmake")
else()
    message(STATUS "Installed SDK is up to date")
endif()

# Samples excluded on Windows. GUI/GPU samples do not work in headless CI.
# NOTE: articulation_chain_with_viewer is already in SAMPLES_EXCLUDED_ALWAYS.
set(SAMPLES_EXCLUDED_WINDOWS
    tensor_bindings_gpu_c
)

# Samples excluded on all platforms. They are debug or reference-only and not part of CI validation.
set(SAMPLES_EXCLUDED_ALWAYS
    articulation_chain_with_viewer
    hello_world_source_link    # uses add_subdirectory, tested separately by test_source_link.cmake
)

set(C_SAMPLES_DIR "${PROJECT_ROOT}/tests/c_samples")
set(C_SAMPLES_INTERNAL_DIR "${PROJECT_ROOT}/tests/c_samples_internal")
if(NOT EXISTS "${C_SAMPLES_DIR}")
    message(FATAL_ERROR "C samples directory not found at ${C_SAMPLES_DIR}")
endif()

file(GLOB SAMPLE_DIRS "${C_SAMPLES_DIR}/*")
if(EXISTS "${C_SAMPLES_INTERNAL_DIR}")
    file(GLOB INTERNAL_SAMPLE_DIRS "${C_SAMPLES_INTERNAL_DIR}/*")
    list(APPEND SAMPLE_DIRS ${INTERNAL_SAMPLE_DIRS})
endif()

# Build with the packaged toolchain the SDK was built with, not whatever Visual
# Studio the machine has. Resolved once. This also exports INCLUDE/LIB/PATH.
set(SAMPLE_TOOLCHAIN_ARGS "")
if(NOT SAMPLES_RUN_ONLY)
    ovphysx_pin_packaged_host_toolchain(SAMPLE_TOOLCHAIN_ARGS)
endif()

set(SAMPLES_RUN 0)
set(SAMPLES_PASSED 0)
set(SAMPLES_FAILED 0)

foreach(SAMPLE_DIR ${SAMPLE_DIRS})
    if(NOT IS_DIRECTORY "${SAMPLE_DIR}")
        continue()
    endif()
    get_filename_component(SAMPLE_NAME "${SAMPLE_DIR}" NAME)

    if(NOT EXISTS "${SAMPLE_DIR}/CMakeLists.txt")
        message(STATUS "Skipping ${SAMPLE_NAME} (no CMakeLists.txt)")
        continue()
    endif()
    
    if(SAMPLE_NAME IN_LIST SAMPLES_EXCLUDED_ALWAYS)
        message(STATUS "Skipping ${SAMPLE_NAME} (excluded)")
        continue()
    endif()
    if(WIN32 AND SAMPLE_NAME IN_LIST SAMPLES_EXCLUDED_WINDOWS)
        message(STATUS "Skipping ${SAMPLE_NAME} (excluded on Windows)")
        continue()
    endif()
    
    if(SAMPLES_BUILD_ONLY)
        message(STATUS "Building ${SAMPLE_NAME} (run skipped)...")
    elseif(SAMPLES_RUN_ONLY)
        message(STATUS "Running prebuilt ${SAMPLE_NAME}...")
    else()
        message(STATUS "Building and running ${SAMPLE_NAME}...")
    endif()
    math(EXPR SAMPLES_RUN "${SAMPLES_RUN} + 1")

    get_filename_component(_PARENT_DIR "${SAMPLE_DIR}" DIRECTORY)
    get_filename_component(_PARENT_NAME "${_PARENT_DIR}" NAME)
    set(SAMPLE_BUILD_DIR "${PROJECT_ROOT}/_build/sample_tests/${_PARENT_NAME}/${SAMPLE_NAME}")
    if(NOT SAMPLES_RUN_ONLY)
        file(REMOVE_RECURSE "${SAMPLE_BUILD_DIR}")
        file(MAKE_DIRECTORY "${SAMPLE_BUILD_DIR}")

        # The external ovstage goes on the prefix path so ovphysxConfig.cmake's
        # find_dependency(ovstage) resolves it, exactly as a consumer would.
        # The ';' is escaped so the list separator survives the argv.
        set(SAMPLE_CMAKE_ARGS
            -DCMAKE_BUILD_TYPE=${BUILD_TYPE}
            "-DCMAKE_PREFIX_PATH=${PROJECT_ROOT}/_install\\;${OVPHYSX_OVSTAGE_ROOT}"
            ${SAMPLE_TOOLCHAIN_ARGS}
        )

        # GCC/Clang-specific flags.
        if(NOT WIN32)
            list(APPEND SAMPLE_CMAKE_ARGS -DCMAKE_CXX_FLAGS=-Wno-changes-meaning)
        endif()

        execute_process(
            COMMAND ${CMAKE_COMMAND} "${SAMPLE_DIR}" ${SAMPLE_CMAKE_ARGS}
            WORKING_DIRECTORY "${SAMPLE_BUILD_DIR}"
            RESULT_VARIABLE CONFIG_RESULT
        )
        if(NOT CONFIG_RESULT STREQUAL "0")
            message(FATAL_ERROR "${SAMPLE_NAME} configuration failed")
            math(EXPR SAMPLES_FAILED "${SAMPLES_FAILED} + 1")
            continue()
        endif()

        # Parallelism is bounded by RAM so a wide sample rebuild does not exhaust memory.
        ovphysx_compute_build_jobs(_SAMPLE_JOBS)
        execute_process(
            COMMAND ${CMAKE_COMMAND} --build . --parallel ${_SAMPLE_JOBS} --config ${BUILD_TYPE}
            WORKING_DIRECTORY "${SAMPLE_BUILD_DIR}"
            RESULT_VARIABLE BUILD_RESULT
        )
        if(NOT BUILD_RESULT STREQUAL "0")
            message(FATAL_ERROR "${SAMPLE_NAME} build failed")
            math(EXPR SAMPLES_FAILED "${SAMPLES_FAILED} + 1")
            continue()
        endif()
    endif()

    if(SAMPLES_BUILD_ONLY)
        message(STATUS "  [OK] ${SAMPLE_NAME} built (run deferred to the test job)")
        math(EXPR SAMPLES_PASSED "${SAMPLES_PASSED} + 1")
        continue()
    endif()

    file(GLOB_RECURSE SAMPLE_EXECUTABLE_LIST "${SAMPLE_BUILD_DIR}/*${SAMPLE_NAME}${EXE_SUFFIX}")
    if(NOT SAMPLE_EXECUTABLE_LIST)
        message(FATAL_ERROR "${SAMPLE_NAME} executable not found in ${SAMPLE_BUILD_DIR}")
        math(EXPR SAMPLES_FAILED "${SAMPLES_FAILED} + 1")
        continue()
    endif()
    list(GET SAMPLE_EXECUTABLE_LIST 0 SAMPLE_EXECUTABLE)
    message(STATUS "  Found executable: ${SAMPLE_EXECUTABLE}")
    
    # Run with a minimal PATH of system directories plus the CUDA Toolkit, if
    # installed. This verifies the SDK does not depend on DLLs from the build
    # tree while still allowing system libraries (WinSock, CryptoAPI, etc.) and
    # the CUDA runtime that GPU samples and their dependencies need.
    if(WIN32)
        # PATH segments are assembled as a CMake list and joined once. This keeps
        # the loader search order explicit and avoids hand-escaped semicolon bugs
        # when passing PATH through `cmake -E env`.
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
        if(DEFINED ENV{CUDA_PATH})
            list(APPEND _WINDOWS_PATH_SEGMENTS "$ENV{CUDA_PATH}\\bin")
        endif()
        list(JOIN _WINDOWS_PATH_SEGMENTS ";" _WINDOWS_PATH_VALUE)
        # `cmake -E env` receives each CMake list item as a separate argv entry.
        # Escaping the semicolons keeps the full Windows PATH as one `PATH=...`
        # argument instead of splitting it into broken pieces.
        string(REPLACE ";" "\\;" _WINDOWS_PATH_VALUE_ESCAPED "${_WINDOWS_PATH_VALUE}")
        set(_ENV_OVERRIDE "PATH=${_WINDOWS_PATH_VALUE_ESCAPED}")
    else()
        set(_ENV_OVERRIDE "LD_LIBRARY_PATH=")
    endif()

    # The OmniPVD sample intentionally retains its recording for inspection, so
    # it runs in the disposable sample build tree instead of the source tree.
    set(SAMPLE_WORKING_DIRECTORY "${PROJECT_ROOT}")
    if(SAMPLE_NAME STREQUAL "omnipvd_recording_cpp")
        set(SAMPLE_WORKING_DIRECTORY "${SAMPLE_BUILD_DIR}")
        file(GLOB OMNIPVD_SOURCE_OUTPUTS_BEFORE
            LIST_DIRECTORIES true
            "${PROJECT_ROOT}/ovphysx_pvd_cpp_sample_*"
        )
    endif()

    execute_process(
        COMMAND ${CMAKE_COMMAND} -E env ${_ENV_OVERRIDE} "${SAMPLE_EXECUTABLE}"
        WORKING_DIRECTORY "${SAMPLE_WORKING_DIRECTORY}"
        TIMEOUT 60
        RESULT_VARIABLE SAMPLE_RESULT
    )

    if(SAMPLE_NAME STREQUAL "omnipvd_recording_cpp")
        file(GLOB OMNIPVD_SOURCE_OUTPUTS_AFTER
            LIST_DIRECTORIES true
            "${PROJECT_ROOT}/ovphysx_pvd_cpp_sample_*"
        )
        foreach(OMNIPVD_SOURCE_OUTPUT ${OMNIPVD_SOURCE_OUTPUTS_AFTER})
            if(NOT OMNIPVD_SOURCE_OUTPUT IN_LIST OMNIPVD_SOURCE_OUTPUTS_BEFORE)
                message(FATAL_ERROR
                    "omnipvd_recording_cpp left output in the source tree: ${OMNIPVD_SOURCE_OUTPUT}"
                )
            endif()
        endforeach()
    endif()
    
    if(NOT SAMPLE_RESULT STREQUAL "0")
        message(FATAL_ERROR "${SAMPLE_NAME} failed (exit code: ${SAMPLE_RESULT})")
        math(EXPR SAMPLES_FAILED "${SAMPLES_FAILED} + 1")
    else()
        message(STATUS "\n  [PASS] ${SAMPLE_NAME} passed")
        math(EXPR SAMPLES_PASSED "${SAMPLES_PASSED} + 1")
    endif()
    
endforeach()

message(STATUS "")
message(STATUS "C/C++ Sample Applications Summary: ${SAMPLES_RUN} run, ${SAMPLES_PASSED} passed, ${SAMPLES_FAILED} failed")
if(SAMPLES_FAILED GREATER 0)
    message(FATAL_ERROR "Some sample applications failed")
endif()

message(STATUS "")
message(STATUS "C++ sample tests: PASSED")

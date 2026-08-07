# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

# ovphysx C++ Unit Tests
# Runs GTest-based C++ unit tests against the INSTALLED SDK
# Usage: cmake -P scripts/test_cpp.cmake
#
# The device mode (CPU vs GPU) and lifecycle refs are process-global. Tests run
# in separate processes so those locks and refs start clean for each pass:
#   Pass 1: GPU tests              (filter: *GpuTest*)          -- GPU lock
#   Pass 2: Global lifecycle tests (filter: GlobalLifecycle.*)  -- clean refs
#   Pass 3: Non-GPU tests          (filter: -*GpuTest*:GlobalLifecycle.*) -- CPU lock
#
# Prerequisite: cmake -P scripts/build.cmake && cmake -P scripts/install.cmake

cmake_minimum_required(VERSION 3.16)

# Setup (self-contained)
get_filename_component(SCRIPT_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
include("${SCRIPT_DIR}/crossplatform_helpers.cmake")
include("${SCRIPT_DIR}/build_common.cmake")

# Paths
set(INSTALL_DIR "${PROJECT_ROOT}/_install")
set(INSTALL_PLUGINS_DIR "${INSTALL_DIR}/plugins")
set(INSTALL_GPU_PLUGINS_DIR "${INSTALL_DIR}/plugins/gpu")

# Verify install directory exists
if(NOT EXISTS "${INSTALL_DIR}")
    message(FATAL_ERROR "_install directory does not exist.\nRun: cmake -P scripts/build.cmake && cmake -P scripts/install.cmake")
endif()

if(NOT EXISTS "${INSTALL_PLUGINS_DIR}")
    message(FATAL_ERROR "_install/plugins directory does not exist.\nRun: cmake -P scripts/install.cmake")
endif()

# Locate GTest executable (still in _build)
set(GTEST_EXECUTABLE "${STANDARD_OUTPUT_DIR}/c_unittests${EXE_SUFFIX}")
set(SIDECAR_GTEST_EXECUTABLE "${STANDARD_OUTPUT_DIR}/sidecar_token_scope_unittests${EXE_SUFFIX}")

if(NOT EXISTS "${GTEST_EXECUTABLE}")
    message(FATAL_ERROR "C++ test executable not found at: ${GTEST_EXECUTABLE}\nRun: cmake -P scripts/build.cmake")
endif()
if(NOT EXISTS "${SIDECAR_GTEST_EXECUTABLE}")
    message(FATAL_ERROR "Sidecar C++ test executable not found at: ${SIDECAR_GTEST_EXECUTABLE}\nRun: cmake -P scripts/build.cmake")
endif()

# Windows: DLLs must be on PATH before process launch (loader resolves before main()).
# Linux: RPATH handles everything, no env setup needed.
# Other env vars (PYTHONPATH) are handled by
# test_main.cpp::bootstrap_test_environment() on both platforms.
if(OS_NAME STREQUAL "windows")
    set(INSTALL_BIN_DIR "${INSTALL_DIR}/bin")
    set(INSTALL_BIN_DEPS_DIR "${INSTALL_PLUGINS_DIR}/bin/deps")
    set(TARGET_PYTHON_DIR "${PROJECT_ROOT}/_build/target-deps/python")
    ovphysx_resolve_ovstage_paths()
    set(BASE_TEST_PATH "${INSTALL_BIN_DIR}${PATH_SEP}${INSTALL_PLUGINS_DIR}${PATH_SEP}${INSTALL_BIN_DEPS_DIR}${PATH_SEP}${TARGET_PYTHON_DIR}${PATH_SEP}${OVPHYSX_OVSTAGE_RUNTIME_DIR}${PATH_SEP}$ENV{PATH}")
    set(GPU_TEST_PATH "${INSTALL_GPU_PLUGINS_DIR}${PATH_SEP}${BASE_TEST_PATH}")
    set(ENV{PATH} "${BASE_TEST_PATH}")
    set(ENV{OVPHYSX_LIB} "${INSTALL_BIN_DIR}/ovphysx.dll")
endif()

message(STATUS "")
message(STATUS "=== C++ Unit Tests (using INSTALLED SDK) ===")
message(STATUS "  Project root: ${PROJECT_ROOT}")
message(STATUS "  Install dir: ${INSTALL_DIR}")
message(STATUS "  Plugins dir: ${INSTALL_PLUGINS_DIR}")
message(STATUS "  Test executable: ${GTEST_EXECUTABLE}")

# -------------------------------------------------------------------------
# Helper functions: run one GTest pass, print tail, check XML & exit code.
# Variables keep the _P_ prefix for readability even though function scope isolates them.
# -------------------------------------------------------------------------
set(TEST_RESULTS_DIR "${BUILD_PATH}/test_results")
file(MAKE_DIRECTORY "${TEST_RESULTS_DIR}")

function(parse_gtest_results _P_LABEL _P_XML)
    file(READ "${_P_XML}" _P_XML_CONTENT)

    foreach(_P_ATTR tests failures disabled skipped)
        string(REGEX MATCH "<testsuites[^>]*${_P_ATTR}=\"([0-9]+)\"" _P_MATCH "${_P_XML_CONTENT}")
        if(NOT _P_MATCH)
            string(REGEX MATCH "<testsuite[^>]*${_P_ATTR}=\"([0-9]+)\"" _P_MATCH "${_P_XML_CONTENT}")
        endif()
        if(_P_MATCH)
            string(REGEX REPLACE ".*${_P_ATTR}=\"([0-9]+)\".*" "\\1" _P_${_P_ATTR} "${_P_MATCH}")
        elseif(_P_ATTR STREQUAL "tests")
            message(FATAL_ERROR "GTest (${_P_LABEL}) results XML is malformed: missing '${_P_ATTR}' count")
        else()
            set(_P_${_P_ATTR} 0)
        endif()
    endforeach()

    if(_P_tests EQUAL 0)
        message(FATAL_ERROR "GTest (${_P_LABEL}) ran zero tests")
    endif()

    math(EXPR _P_passed "${_P_tests} - ${_P_failures} - ${_P_disabled} - ${_P_skipped}")
    if(_P_passed LESS 0)
        message(FATAL_ERROR "GTest (${_P_LABEL}) results XML has inconsistent counts")
    endif()

    message(STATUS
        "GTest ${_P_LABEL}: ${_P_tests} tests, ${_P_passed} passed, ${_P_skipped} skipped, "
        "${_P_disabled} disabled, ${_P_failures} failed")

    if(_P_failures GREATER 0)
        message(FATAL_ERROR "C++ unit tests had failures in ${_P_LABEL} pass")
    endif()
endfunction()

function(run_gtest_pass _P_LABEL _P_FILTER)
    set(_P_EXECUTABLE "${GTEST_EXECUTABLE}")
    if(ARGC GREATER 2)
        set(_P_EXECUTABLE "${ARGV2}")
    endif()
    set(_P_XML "${TEST_RESULTS_DIR}/gtest_${_P_LABEL}_results.xml")
    # Single combined log (stdout + stderr interleaved) for easier triage.
    set(_P_LOG "${TEST_RESULTS_DIR}/gtest_${_P_LABEL}.log")
    file(REMOVE "${_P_XML}" "${_P_LOG}")

    message(STATUS "")
    message(STATUS "--- ${_P_LABEL} pass (filter: ${_P_FILTER}) ---")

    # NOTE: OUTPUT_FILE + ERROR_FILE pointing to the same path merges both streams.
    set(_P_ENV)
    set(_P_ORIGINAL_PATH "$ENV{PATH}")
    if("${_P_LABEL}" STREQUAL "gpu")
        list(APPEND _P_ENV "OVPHYSX_TEST_REQUIRE_CUDA=1")
        if(OS_NAME STREQUAL "windows")
            set(ENV{PATH} "${GPU_TEST_PATH}")
        endif()
    elseif("${_P_LABEL}" STREQUAL "lifecycle")
        list(APPEND _P_ENV "OVPHYSX_DISABLE_GPU=1")
    elseif("${_P_LABEL}" STREQUAL "cpu")
        list(APPEND _P_ENV "OVPHYSX_DISABLE_GPU=1")
        if(OS_NAME STREQUAL "windows")
            set(ENV{PATH} "${BASE_TEST_PATH}")
        endif()
    elseif("${_P_LABEL}" STREQUAL "sidecar-token-scope")
        list(APPEND _P_ENV "OVPHYSX_DISABLE_GPU=1")
        if(OS_NAME STREQUAL "windows")
            set(ENV{PATH} "${BASE_TEST_PATH}")
        endif()
    endif()
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E env ${_P_ENV} ${_P_EXECUTABLE}
                "--gtest_output=xml:${_P_XML}"
                "--gtest_filter=${_P_FILTER}"
        WORKING_DIRECTORY "${PROJECT_ROOT}"
        TIMEOUT 300
        RESULT_VARIABLE _P_RC
        OUTPUT_VARIABLE _P_STDOUT
        ERROR_VARIABLE  _P_STDERR
        ECHO_OUTPUT_VARIABLE
        ECHO_ERROR_VARIABLE
    )
    if(OS_NAME STREQUAL "windows")
        set(ENV{PATH} "${_P_ORIGINAL_PATH}")
    endif()

    file(WRITE "${_P_LOG}" "${_P_STDOUT}${_P_STDERR}")

    message(STATUS "Exit code (${_P_LABEL}): ${_P_RC}")

    # Print tail for quick CI triage (full output in the log file).
    if(EXISTS "${_P_LOG}")
        file(READ "${_P_LOG}" _P_LOG_ALL)
        string(LENGTH "${_P_LOG_ALL}" _P_LOG_LEN)
        math(EXPR _P_LOG_START "(${_P_LOG_LEN} - 8192)")
        if(_P_LOG_START LESS 0)
            set(_P_LOG_START 0)
        endif()
        string(SUBSTRING "${_P_LOG_ALL}" ${_P_LOG_START} -1 _P_LOG_TAIL)
        if(_P_LOG_TAIL)
            message(STATUS "GTest ${_P_LABEL} output tail (last ~8KB):\n${_P_LOG_TAIL}")
        endif()
    endif()

    # XML must exist (proves tests ran without crashing).
    if(EXISTS "${_P_XML}")
        parse_gtest_results("${_P_LABEL}" "${_P_XML}")
    else()
        message(STATUS "GTest log: ${_P_LOG}")
        message(FATAL_ERROR
            "GTest (${_P_LABEL}) completed but XML output not found. "
            "Tests likely crashed (exit code: ${_P_RC})")
    endif()

    # Non-zero exit means at least one test failed.
    if(NOT "${_P_RC}" STREQUAL "0")
        message(STATUS "GTest log: ${_P_LOG}")
        message(FATAL_ERROR "C++ unit tests failed in ${_P_LABEL} pass (exit code: ${_P_RC})")
    endif()
endfunction()

# -------------------------------------------------------------------------
# Pass 1 - GPU tests (separate process; if CUDA is present tests will use GPU).
# Exclude CpuNoCudaContextGpuTest: it must run in a process where no GPU test
# has yet created a CUDA context (see Pass 3), so it is routed to its own pass.
# -------------------------------------------------------------------------
run_gtest_pass("gpu" "*GpuTest*:-CpuNoCudaContextGpuTest.*")

# -------------------------------------------------------------------------
# Pass 2 - Global lifecycle tests (own process so runtime refs start clean)
# -------------------------------------------------------------------------
run_gtest_pass("lifecycle" "GlobalLifecycle.*")

# -------------------------------------------------------------------------
# Pass 3 - Non-GPU tests
# -------------------------------------------------------------------------
run_gtest_pass("cpu" "-*GpuTest*:GlobalLifecycle.*")

# -------------------------------------------------------------------------
# Pass 4 - CPU-no-CUDA-context contract (own fresh process). On a GPU box, a
# cpu_only instance must not open a CUDA context. Isolated so the primary-context
# probe starts from a clean state; self-skips on CPU-only runners.
# -------------------------------------------------------------------------
run_gtest_pass("cpu-no-cuda-context" "CpuNoCudaContextGpuTest.*")

# -------------------------------------------------------------------------
# Pass 5 - isolated checked token-scope sidecar bridge tests.
# -------------------------------------------------------------------------
run_gtest_pass("sidecar-token-scope" "*" "${SIDECAR_GTEST_EXECUTABLE}")

# -------------------------------------------------------------------------
# Check for simulation stage leaks across both passes
# -------------------------------------------------------------------------
set(GTEST_ALL_OUTPUT "")
foreach(_LEAK_LOG
    "${TEST_RESULTS_DIR}/gtest_gpu.log"
    "${TEST_RESULTS_DIR}/gtest_lifecycle.log"
    "${TEST_RESULTS_DIR}/gtest_cpu.log"
    "${TEST_RESULTS_DIR}/gtest_cpu-no-cuda-context.log"
    "${TEST_RESULTS_DIR}/gtest_sidecar-token-scope.log")
    if(EXISTS "${_LEAK_LOG}")
        file(READ "${_LEAK_LOG}" _LEAK_CONTENTS)
        string(APPEND GTEST_ALL_OUTPUT "${_LEAK_CONTENTS}")
    endif()
endforeach()
if(GTEST_ALL_OUTPUT MATCHES "outstanding SimStageWithHistory")
    string(REGEX MATCH "had ([0-9]+) outstanding" LEAK_MATCH "${GTEST_ALL_OUTPUT}")
    if(CMAKE_MATCH_1)
        message(FATAL_ERROR "Simulation stage leak detected: ${CMAKE_MATCH_1} outstanding SimStageWithHistory(s) at shutdown")
    else()
        message(FATAL_ERROR "Simulation stage leak detected (see test output)")
    endif()
endif()

message(STATUS "")
message(STATUS "C++ unit tests: PASSED")

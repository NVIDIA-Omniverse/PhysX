# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# ovphysx C++ unit tests.
# Runs the GTest-based C++ unit tests against the installed SDK.
# Usage: cmake -P scripts/test_cpp.cmake
#
# The device mode (CPU vs GPU) and lifecycle refs are process-global. Tests run
# in separate processes so those locks and refs start clean for each pass:
#   Attach-time CUDA selection test (isolated process)         clean selector
#   Remaining GPU tests       (filter: *GpuTest*)               GPU lock
#   Global lifecycle tests    (filter: GlobalLifecycle.*)       clean refs
#   Non-GPU tests             (filter: -*GpuTest*:GlobalLifecycle.*) CPU lock
#
# Prerequisite: cmake -P scripts/build.cmake && cmake -P scripts/install.cmake

cmake_minimum_required(VERSION 3.16)

get_filename_component(SCRIPT_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
include("${SCRIPT_DIR}/crossplatform_helpers.cmake")
include("${SCRIPT_DIR}/build_common.cmake")

set(INSTALL_DIR "${PROJECT_ROOT}/_install")
set(INSTALL_PLUGINS_DIR "${INSTALL_DIR}/plugins")

if(NOT EXISTS "${INSTALL_DIR}")
    message(FATAL_ERROR "_install directory does not exist.\nRun: cmake -P scripts/build.cmake && cmake -P scripts/install.cmake")
endif()

if(NOT EXISTS "${INSTALL_PLUGINS_DIR}")
    message(FATAL_ERROR "_install/plugins directory does not exist.\nRun: cmake -P scripts/install.cmake")
endif()

# The test executables stay in _build. Only the SDK under test is installed.
set(GTEST_EXECUTABLE "${STANDARD_OUTPUT_DIR}/c_unittests${EXE_SUFFIX}")
set(SIDECAR_GTEST_EXECUTABLE "${STANDARD_OUTPUT_DIR}/sidecar_token_scope_unittests${EXE_SUFFIX}")

if(NOT EXISTS "${GTEST_EXECUTABLE}")
    message(FATAL_ERROR "C++ test executable not found at: ${GTEST_EXECUTABLE}\nRun: cmake -P scripts/build.cmake")
endif()
if(NOT EXISTS "${SIDECAR_GTEST_EXECUTABLE}")
    message(FATAL_ERROR "Sidecar C++ test executable not found at: ${SIDECAR_GTEST_EXECUTABLE}\nRun: cmake -P scripts/build.cmake")
endif()

# On Windows the DLLs must be on PATH before process launch, because the loader
# resolves them before main(). On Linux RPATH handles everything. Other env vars
# (PYTHONPATH) are set by test_main.cpp::bootstrap_test_environment() on both platforms.
if(OS_NAME STREQUAL "windows")
    set(INSTALL_BIN_DIR "${INSTALL_DIR}/bin")
    set(INSTALL_BIN_DEPS_DIR "${INSTALL_PLUGINS_DIR}/bin/deps")
    set(TARGET_PYTHON_DIR "${PROJECT_ROOT}/_build/target-deps/python")
    ovphysx_resolve_ovstage_paths()
    # OVStage's plugins dir carries the USD monolith, which the SDK does not
    # ship (REQ-PACKAGING-USDFREE-001 AC-4) and sidecar_token_scope_unittests imports
    # directly through ovphysx_internal.
    set(BASE_TEST_PATH "${INSTALL_BIN_DIR}${PATH_SEP}${INSTALL_PLUGINS_DIR}${PATH_SEP}${INSTALL_BIN_DEPS_DIR}${PATH_SEP}${TARGET_PYTHON_DIR}${PATH_SEP}${OVPHYSX_OVSTAGE_RUNTIME_DIR}${PATH_SEP}${OVPHYSX_OVSTAGE_RUNTIME_DIR}/plugins${PATH_SEP}$ENV{PATH}")
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
# Helper functions: run one GTest pass, print the tail, check the XML and exit code.
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
    set(_P_TIMEOUT 300)
    if(ARGC GREATER 2)
        set(_P_EXECUTABLE "${ARGV2}")
    endif()
    if(ARGC GREATER 3)
        set(_P_TIMEOUT "${ARGV3}")
    endif()
    set(_P_XML "${TEST_RESULTS_DIR}/gtest_${_P_LABEL}_results.xml")
    # One combined log with stdout and stderr interleaved, for easier triage.
    set(_P_LOG "${TEST_RESULTS_DIR}/gtest_${_P_LABEL}.log")
    file(REMOVE "${_P_XML}" "${_P_LOG}")

    message(STATUS "")
    message(STATUS "--- ${_P_LABEL} pass (filter: ${_P_FILTER}) ---")

    set(_P_ENV)
    set(_P_ORIGINAL_PATH "$ENV{PATH}")
    if("${_P_LABEL}" STREQUAL "gpu")
        list(APPEND _P_ENV "OVPHYSX_TEST_REQUIRE_CUDA=1")
    elseif("${_P_LABEL}" MATCHES "^lifecycle")
        list(APPEND _P_ENV "OVPHYSX_DISABLE_GPU=1")
        list(APPEND _P_ENV "OVPHYSX_TEST_LIFECYCLE_OWNS_INIT=1")
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
        TIMEOUT ${_P_TIMEOUT}
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

    # The tail is printed for quick CI triage. The full output is in the log file.
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

    if("${_P_RC}" MATCHES "[Tt]imeout")
        message(STATUS "GTest log: ${_P_LOG}")
        message(FATAL_ERROR
            "GTest (${_P_LABEL}) timed out after ${_P_TIMEOUT}s; log: ${_P_LOG}")
    endif()

    # The XML proves the test process reached the result writer.
    if(EXISTS "${_P_XML}")
        parse_gtest_results("${_P_LABEL}" "${_P_XML}")
    else()
        message(STATUS "GTest log: ${_P_LOG}")
        message(FATAL_ERROR
            "GTest (${_P_LABEL}) produced no XML output. "
            "The process likely crashed or failed to start (exit code: ${_P_RC})")
    endif()

    # A non-zero exit with valid XML points at a teardown failure.
    if(NOT "${_P_RC}" STREQUAL "0")
        message(STATUS "GTest log: ${_P_LOG}")
        message(FATAL_ERROR
            "GTest (${_P_LABEL}) wrote valid XML but exited non-zero afterward. "
            "This may be a teardown crash (exit code: ${_P_RC})")
    endif()
endfunction()

# -------------------------------------------------------------------------
# Attach-time CUDA selection runs in a fresh process so the selector starts at
# automatic (-1) before the OVStage attach under test.
# -------------------------------------------------------------------------
run_gtest_pass(
    "cuda-selection-attach"
    "ActiveCudaGpusAttachTest.*:-ActiveCudaGpusAttachTest.DirectGpuHostReadOnNonZeroOrdinal")

# -------------------------------------------------------------------------
# The non-zero-ordinal DirectGPU read needs a process with no earlier GPU attach.
# PhysX latches its CUDA device at the first GPU attach in the process, so an
# attach on ordinal 0 pins the device and the later /physics/cudaDevice = 1 write
# does not move it. Sharing the pass above makes the case run on ordinal 0.
# Self-skips on single-GPU runners.
# -------------------------------------------------------------------------
run_gtest_pass(
    "cuda-selection-nonzero-ordinal"
    "ActiveCudaGpusAttachTest.DirectGpuHostReadOnNonZeroOrdinal")

# -------------------------------------------------------------------------
# Remaining GPU tests. The CPU-no-CUDA-context test also requires a fresh
# process and is routed to its own pass below.
# -------------------------------------------------------------------------
run_gtest_pass("gpu" "*GpuTest*:-CpuNoCudaContextGpuTest.*")

# -------------------------------------------------------------------------
# Global lifecycle tests run in their own process so runtime refs start clean.
# The callback-drain regressions get a separate short-timeout process. A broken
# drain or lock hoist would otherwise wedge callback and shutdown threads, so
# in-process cleanup cannot safely recover.
# -------------------------------------------------------------------------
run_gtest_pass(
    "lifecycle"
    "GlobalLifecycle.*:-GlobalLifecycle.ShutdownDrainsAcceptedCallback:GlobalLifecycle.LiveInstanceShutdownDrainDoesNotHoldInstanceMapLock")
run_gtest_pass(
    "lifecycle-lock-drain"
    "GlobalLifecycle.ShutdownDrainsAcceptedCallback:GlobalLifecycle.LiveInstanceShutdownDrainDoesNotHoldInstanceMapLock"
    "${GTEST_EXECUTABLE}"
    60)

# Provider creation is process-latched, so this exclusion regression needs a
# fresh process before any other test can create PxPhysics.
run_gtest_pass("omnipvd-cold" "OmniPvdColdCreation.*")

# -------------------------------------------------------------------------
# Non-GPU tests
# -------------------------------------------------------------------------
run_gtest_pass("cpu" "-*GpuTest*:GlobalLifecycle.*:ActiveCudaGpusAttachTest.*:OmniPvdColdCreation.*")

# -------------------------------------------------------------------------
# CPU-no-CUDA-context contract. On a GPU box, a cpu_only instance must not open
# a CUDA context. Runs in a fresh process so the primary-context probe starts
# from a clean state. Self-skips on CPU-only runners.
# -------------------------------------------------------------------------
run_gtest_pass("cpu-no-cuda-context" "CpuNoCudaContextGpuTest.*")

# -------------------------------------------------------------------------
# Isolated checked token-scope sidecar bridge tests.
# -------------------------------------------------------------------------
run_gtest_pass("sidecar-token-scope" "*" "${SIDECAR_GTEST_EXECUTABLE}")

# -------------------------------------------------------------------------
# Check for simulation stage leaks across all passes.
# -------------------------------------------------------------------------
set(GTEST_ALL_OUTPUT "")
foreach(_LEAK_LOG
    "${TEST_RESULTS_DIR}/gtest_cuda-selection-attach.log"
    "${TEST_RESULTS_DIR}/gtest_cuda-selection-nonzero-ordinal.log"
    "${TEST_RESULTS_DIR}/gtest_gpu.log"
    "${TEST_RESULTS_DIR}/gtest_lifecycle.log"
    "${TEST_RESULTS_DIR}/gtest_lifecycle-lock-drain.log"
    "${TEST_RESULTS_DIR}/gtest_omnipvd-cold.log"
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

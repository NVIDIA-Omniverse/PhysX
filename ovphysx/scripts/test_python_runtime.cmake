# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PACKAGING-TESTDEPS-001
# @covers AC-1 AC-2 AC-3
#
# @implements REQ-PACKAGING-PYTESTUSD-001
# @covers AC-2 AC-3
#
# @implements REQ-PYTHON-UTILS-001
# @covers AC-10 AC-14
#
# ovphysx Python Tests
# Runs pytest-based Python unit tests against the INSTALLED SDK (_install/)
# Usage: cmake -P scripts/test_python_runtime.cmake
#
# NOTE: This script runs in an isolated process (via execute_process in validate_all.cmake).
# All test scripts run in isolated processes to avoid library conflicts between:
# - C++ tests that load Carbonite plugins with embedded Python
# - Python tests that pre-load 116+ native libraries with RTLD_GLOBAL

cmake_minimum_required(VERSION 3.16)
get_filename_component(SCRIPT_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
include("${SCRIPT_DIR}/crossplatform_helpers.cmake")
include("${SCRIPT_DIR}/build_common.cmake")
include("${SCRIPT_DIR}/parallel_execute.cmake")

set(INSTALL_DIR "${PROJECT_ROOT}/_install")

# ovstage is not installed, so the configured external package is used.
ovphysx_resolve_ovstage_paths()
set(OVSTAGE_PYTHON_DIR "${OVPHYSX_OVSTAGE_PYTHON_DIR}")
if(NOT OVSTAGE_PYTHON_DIR)
    message(FATAL_ERROR
        "The configured ovstage root '${OVPHYSX_OVSTAGE_ROOT}' carries no importable "
        "ovstage Python package; the Python runtime tests need one.")
endif()

if(NOT OVPHYSX_PYTHON_RUNTIME_INNER)
    # The python tests resolve python USD from stock pip usd-core in the uv venv
    # (REQ-PACKAGING-PYTESTUSD-001); the main session's only pxr user
    # (test_documentation_contracts.py) opens a .usda in a clean subprocess, and the
    # pure-pxr utils_tests/ suite runs in its own pytest process below, so usd-core is
    # never resident next to ovstage. Run the test body in a child process so a native
    # abort in the suite cannot take down this parent.
    set(_PYTHON_RUNTIME_CHILD_COMMAND
        "${CMAKE_COMMAND}"
        "-DBUILD_TYPE=${BUILD_TYPE}"
        "-DOVPHYSX_UV_COMMAND=${OVPHYSX_UV_COMMAND}"
        -DOVPHYSX_PYTHON_RUNTIME_INNER=ON
        -P "${CMAKE_CURRENT_LIST_FILE}")
    execute_process(
        COMMAND ${_PYTHON_RUNTIME_CHILD_COMMAND}
        RESULT_VARIABLE _PYTHON_RUNTIME_CHILD_RESULT)
    if(NOT "${_PYTHON_RUNTIME_CHILD_RESULT}" STREQUAL "0")
        message(FATAL_ERROR
            "Python runtime test child failed (exit code: ${_PYTHON_RUNTIME_CHILD_RESULT})")
    endif()
    return()
endif()

function(parse_pytest_junit LABEL XML_PATH)
    if(NOT EXISTS "${XML_PATH}")
        message(FATAL_ERROR "${LABEL} failed to generate results XML: ${XML_PATH}")
    endif()

    file(READ "${XML_PATH}" PYTEST_XML_CONTENT)

    foreach(_ATTR tests failures errors skipped)
        string(REGEX MATCH "<testsuite[^>]*${_ATTR}=\"([0-9]+)\"" _MATCH "${PYTEST_XML_CONTENT}")
        if(NOT _MATCH)
            string(REGEX MATCH "<testsuites[^>]*${_ATTR}=\"([0-9]+)\"" _MATCH "${PYTEST_XML_CONTENT}")
        endif()
        if(_MATCH)
            string(REGEX REPLACE ".*${_ATTR}=\"([0-9]+)\".*" "\\1" _VALUE "${_MATCH}")
        elseif(_ATTR STREQUAL "tests")
            message(FATAL_ERROR "${LABEL} results XML is malformed: missing '${_ATTR}' count")
        else()
            set(_VALUE 0)
        endif()
        set(_${_ATTR} "${_VALUE}")
    endforeach()

    if(_tests EQUAL 0)
        message(FATAL_ERROR "${LABEL} ran zero tests")
    endif()

    math(EXPR _passed "${_tests} - ${_failures} - ${_errors} - ${_skipped}")
    if(_passed LESS 0)
        message(FATAL_ERROR "${LABEL} results XML has inconsistent counts")
    endif()

    message(STATUS
        "${LABEL}: ${_tests} tests, ${_passed} passed, ${_skipped} skipped, "
        "${_failures} failed, ${_errors} errors")

    if(_failures GREATER 0 OR _errors GREATER 0)
        message(FATAL_ERROR "${LABEL} had failures or errors")
    endif()
endfunction()

# The target-deps Python is 3.12.
if(WIN32)
    set(TARGET_PYTHON "${PROJECT_ROOT}/_build/target-deps/python/python.exe")
else()
    set(TARGET_PYTHON "${PROJECT_ROOT}/_build/target-deps/python/bin/python3")
endif()

set(PYTHON_TEST_DIR "${PROJECT_ROOT}/tests/python_tests")
message(STATUS "")
message(STATUS "=== Python Tests (using installed SDK) ===")
message(STATUS "Python test directory: ${PYTHON_TEST_DIR}")
message(STATUS "Install path: ${INSTALL_DIR}")

if(NOT EXISTS "${INSTALL_DIR}/plugins")
    message(FATAL_ERROR
        "Install directory not found at ${INSTALL_DIR}/plugins\n"
        "Run 'cmake -P scripts/install.cmake' first to create the SDK install tree.")
endif()

execute_process(
    COMMAND "${TARGET_PYTHON}" --version
    OUTPUT_VARIABLE PYTHON_VERSION_OUTPUT
    ERROR_VARIABLE PYTHON_VERSION_OUTPUT  # Some Python versions print the version to stderr.
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_STRIP_TRAILING_WHITESPACE
)
string(REGEX MATCH "[0-9]+\\.[0-9]+\\.[0-9]+" PYTHON_VERSION "${PYTHON_VERSION_OUTPUT}")
if(NOT PYTHON_VERSION)
    set(PYTHON_VERSION "unknown")
endif()

execute_process(
    COMMAND "${OVPHYSX_UV_COMMAND}" --version
    OUTPUT_VARIABLE UV_VERSION
    ERROR_QUIET
    RESULT_VARIABLE UV_CHECK
)
if(NOT UV_CHECK EQUAL 0)
    message(FATAL_ERROR "uv not found (needed for running pytest)")
endif()
string(STRIP "${UV_VERSION}" UV_VERSION)
message(STATUS "Found uv: ${UV_VERSION}")

set(VENV_DIR "${PYTHON_TEST_DIR}/.venv")
if(EXISTS "${VENV_DIR}")
    file(REMOVE_RECURSE "${VENV_DIR}")
endif()
message(STATUS "Creating venv with target-deps Python...")
execute_process(
    COMMAND "${TARGET_PYTHON}" -m venv "${VENV_DIR}"
    WORKING_DIRECTORY "${PYTHON_TEST_DIR}"
    RESULT_VARIABLE VENV_RESULT
)
if(NOT VENV_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to create venv with target-deps Python")
endif()

# Previous runs can leave stale editable-install metadata behind.
set(PYTHON_TEST_EGG_INFO "${PYTHON_TEST_DIR}/ovphysx_python_tests.egg-info")
if(EXISTS "${PYTHON_TEST_EGG_INFO}")
    file(REMOVE_RECURSE "${PYTHON_TEST_EGG_INFO}")
    if(EXISTS "${PYTHON_TEST_EGG_INFO}")
        message(FATAL_ERROR "Cannot remove stale egg-info dir: ${PYTHON_TEST_EGG_INFO}. Please fix permissions or delete it.")
    endif()
endif()

set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "1")
# Test results go to _build so the source tree stays clean.
set(PYTEST_RESULTS_DIR "${PROJECT_ROOT}/_build/test_results")
file(MAKE_DIRECTORY "${PYTEST_RESULTS_DIR}")
set(PYTEST_RESULTS_XML "${PYTEST_RESULTS_DIR}/pytest_results.xml")

set(UV_CACHE_DIR_PATH "${PROJECT_ROOT}/_build/.uv_cache")
file(MAKE_DIRECTORY "${UV_CACHE_DIR_PATH}")
set(OVSTAGE_WHEEL_DIR "${PROJECT_ROOT}/_build/target-deps/ovstage_wheel")
file(GLOB OVSTAGE_WHEELS "${OVSTAGE_WHEEL_DIR}/ovstage-*.whl")
if(NOT OVSTAGE_WHEELS)
    message(FATAL_ERROR "ovstage wheel not found in ${OVSTAGE_WHEEL_DIR}. Run the ovphysx build first.")
endif()
# UV_NO_CONFIG=1 discards uv's configuration settings, pyproject.toml's find-links
# among them, so the staged wheel dir is passed here. It is spelled relative to uv's
# working directory, which is also the string uv.lock records as the ovstage registry.
# On uv 0.12.8 a relative find-links resolving to that directory is accepted, while an
# absolute path to the same directory makes uv discard the lock and re-resolve, which
# has no solution against one staged wheel. required-environments is project metadata
# rather than configuration, so UV_NO_CONFIG leaves it in effect and it is what makes
# that re-resolution unsatisfiable. An absolute path must not be reintroduced even if
# the pinned uv happens to tolerate one. That tolerance depends on the checked-in lock,
# not on the uv version.
file(RELATIVE_PATH OVSTAGE_WHEEL_FIND_LINKS "${PYTHON_TEST_DIR}" "${OVSTAGE_WHEEL_DIR}")
set(UV_ENV
    "UV_CACHE_DIR=${UV_CACHE_DIR_PATH}"
    "UV_NO_CONFIG=1"
    "UV_FIND_LINKS=${OVSTAGE_WHEEL_FIND_LINKS}"
    "UV_HTTP_TIMEOUT=300"
    "UV_SKIP_WHEEL_FILENAME_CHECK=1"
)

if(NOT EXISTS "${OVSTAGE_PYTHON_DIR}/ovstage/__init__.py")
    message(FATAL_ERROR
        "ovstage Python package not found at ${OVSTAGE_PYTHON_DIR}/ovstage.\n"
        "Run scripts/fetch_deps.cmake so the ovstage release is extracted.")
endif()

if(WIN32)
    set(OVSTAGE_RUNTIME_DIR "${OVPHYSX_OVSTAGE_RUNTIME_DIR}")
    set(OVSTAGE_RUNTIME_FILE "${OVSTAGE_RUNTIME_DIR}/ovstage.dll")
    # pxr's Tf import wrapper registers every PATH entry with os.add_dll_directory().
    # ovstage's own plugin tree carries the USD DLL closure it needs; stock usd-core
    # loads its own DLLs from the wheel. omni.client.lib is OVStage's, exposed here
    # because ovstage loads OmniClient from its plugin tree, not the ovphysx payload.
    set(_WINDOWS_PATH_SEGMENTS
        "${INSTALL_DIR}/bin"
        "${OVSTAGE_RUNTIME_DIR}"
        "${OVSTAGE_PYTHON_DIR}/ovstage/bin/plugins"
        "${OVSTAGE_PYTHON_DIR}/ovstage/bin/plugins/omni.client.lib"
        "${INSTALL_DIR}/plugins"
        "${OVSTAGE_PYTHON_DIR}/ovstage/bin"
        "$ENV{PATH}")
    list(JOIN _WINDOWS_PATH_SEGMENTS ";" _WINDOWS_PATH_VALUE)
    set(ENV{PATH} "${_WINDOWS_PATH_VALUE}")
    set(ENV{PYTHONPATH} "${OVSTAGE_PYTHON_DIR}")
    set(OVPHYSX_PYTHON_NATIVE_ENV
        "OVSTAGE_LIBRARY_PATH_HINT=${OVSTAGE_RUNTIME_DIR}"
    )
else()
    set(OVSTAGE_RUNTIME_DIR "${OVSTAGE_PYTHON_DIR}/ovstage/bin")
    set(OVSTAGE_RUNTIME_FILE "${OVSTAGE_RUNTIME_DIR}/libovstage.so")
    set(OVPHYSX_PYTHON_NATIVE_ENV
        "PYTHONPATH=${OVSTAGE_PYTHON_DIR}"
        "OVSTAGE_LIBRARY_PATH_HINT=${OVSTAGE_RUNTIME_DIR}"
        "LD_LIBRARY_PATH=${INSTALL_DIR}/lib:${INSTALL_DIR}/plugins:${OVSTAGE_RUNTIME_DIR}:$ENV{LD_LIBRARY_PATH}"
    )
endif()
if(NOT EXISTS "${OVSTAGE_RUNTIME_FILE}")
    message(FATAL_ERROR "ovstage runtime not found at ${OVSTAGE_RUNTIME_FILE}")
endif()
foreach(_OVPHYSX_ENV_ENTRY IN LISTS OVPHYSX_PYTHON_NATIVE_ENV)
    list(APPEND UV_ENV "${_OVPHYSX_ENV_ENTRY}")
endforeach()

# cpu_tests/, lifecycle_tests/ and utils_tests/ run in separate processes below, so
# they are excluded from this whole-suite pass.
message(STATUS "Running pytest...")
execute_process(
    COMMAND ${CMAKE_COMMAND} -E env ${UV_ENV} "${OVPHYSX_UV_COMMAND}" run pytest --ignore=cpu_tests --ignore=lifecycle_tests --ignore=utils_tests --junit-xml=${PYTEST_RESULTS_XML} -v -s
    WORKING_DIRECTORY "${PYTHON_TEST_DIR}"
    RESULT_VARIABLE PYTEST_RESULT
    OUTPUT_VARIABLE PYTEST_STDOUT
    ERROR_VARIABLE PYTEST_STDERR
    ECHO_OUTPUT_VARIABLE
    ECHO_ERROR_VARIABLE
    COMMAND_ECHO STDOUT
    TIMEOUT 900
)

message(STATUS "Pytest result code: ${PYTEST_RESULT}")

# The exit code is checked first so a crash is caught even when the XML was written.
if(NOT PYTEST_RESULT EQUAL 0)
    message(FATAL_ERROR "Python tests failed (exit code: ${PYTEST_RESULT})")
endif()

parse_pytest_junit("Python Tests" "${PYTEST_RESULTS_XML}")

# ============================================================================
# CPU-mode tests (separate pytest invocation)
#
# Carbonite/PhysX device mode is a process-global singleton. The first PhysX()
# call locks in CPU or GPU for the entire process lifetime. The main suite above
# uses explicit GPU-mode fixtures. CPU-mode tests live in cpu_tests/ and MUST run
# in a separate process to get a fresh Carbonite initialization with device="cpu".
# ============================================================================

set(CPU_TEST_DIR "${PYTHON_TEST_DIR}/cpu_tests")
if(EXISTS "${CPU_TEST_DIR}")
    set(PYTEST_CPU_RESULTS_XML "${PYTEST_RESULTS_DIR}/pytest_cpu_results.xml")
    file(REMOVE "${PYTEST_CPU_RESULTS_XML}")

    message(STATUS "")
    message(STATUS "Running CPU-mode pytest (separate process)...")

    execute_process(
        COMMAND ${CMAKE_COMMAND} -E env ${UV_ENV} "${OVPHYSX_UV_COMMAND}" run pytest ${CPU_TEST_DIR} --junit-xml=${PYTEST_CPU_RESULTS_XML} -v -s
        WORKING_DIRECTORY "${PYTHON_TEST_DIR}"
        RESULT_VARIABLE PYTEST_CPU_RESULT
        OUTPUT_VARIABLE PYTEST_CPU_STDOUT
        ERROR_VARIABLE PYTEST_CPU_STDERR
        ECHO_OUTPUT_VARIABLE
        ECHO_ERROR_VARIABLE
        COMMAND_ECHO STDOUT
        TIMEOUT 300
    )

    if(NOT PYTEST_CPU_RESULT EQUAL 0)
        message(FATAL_ERROR "CPU-mode Python tests failed (exit code: ${PYTEST_CPU_RESULT})")
    endif()
    parse_pytest_junit("CPU-mode Python Tests" "${PYTEST_CPU_RESULTS_XML}")
else()
    message(STATUS "No cpu_tests/ directory found, skipping CPU-mode tests")
endif()

# ============================================================================
# ovphysx.utils tests (separate pytest invocation)
#
# USD builds its schema registry once, on first access, so the codeless PhysX
# schemas must be registered before anything in the process opens a stage or
# queries the registry. A registration that arrives later is silently
# ineffective. utils_tests/ therefore needs a process that runs nothing else.
# It needs no PhysX instance and no GPU: the subpackage is pure `pxr`, served by
# the venv's stock usd-core (REQ-PACKAGING-PYTESTUSD-001). The main session above
# ignores this directory, which is what keeps its in-process pxr imports away from
# ovstage's resident USD.
#
# PyPI ships no usd-core wheel for linux-aarch64 (pyproject marker
# `platform_machine != 'aarch64'`, REQ-PACKAGING-PYTESTUSD-001 AC-4), so there the
# venv has no pxr at all: every module here would importorskip, pytest would
# collect nothing and exit 5, and the suite's own REQUIRE guard would refuse to
# run. Do not launch it there; the gap is accepted and recorded in the REQ.
# ============================================================================

set(UTILS_TEST_DIR "${PYTHON_TEST_DIR}/utils_tests")
if(EXISTS "${UTILS_TEST_DIR}" AND ARCH_NAME STREQUAL "aarch64")
    message(STATUS "")
    message(STATUS "Skipping ovphysx.utils pytest: no python USD on linux-aarch64 (usd-core publishes no wheel)")
elseif(EXISTS "${UTILS_TEST_DIR}")
    set(PYTEST_UTILS_RESULTS_XML "${PYTEST_RESULTS_DIR}/pytest_utils_results.xml")
    file(REMOVE "${PYTEST_UTILS_RESULTS_XML}")

    message(STATUS "")
    message(STATUS "Running ovphysx.utils pytest (separate process)...")

    # The suite degrades to skips where the codeless schemas cannot be
    # registered, which is right for a raw source checkout but would take every
    # stage-opening case out of this run and still report a pass. Against a
    # staged install tree an unregistrable schema tree is a broken test stage,
    # so tell the suite to fail instead.
    set(UTILS_ENV ${UV_ENV} "OVPHYSX_REQUIRE_CODELESS_SCHEMAS=1")

    execute_process(
        COMMAND ${CMAKE_COMMAND} -E env ${UTILS_ENV} "${OVPHYSX_UV_COMMAND}" run pytest ${UTILS_TEST_DIR} --junit-xml=${PYTEST_UTILS_RESULTS_XML} -v -s
        WORKING_DIRECTORY "${PYTHON_TEST_DIR}"
        RESULT_VARIABLE PYTEST_UTILS_RESULT
        OUTPUT_VARIABLE PYTEST_UTILS_STDOUT
        ERROR_VARIABLE PYTEST_UTILS_STDERR
        ECHO_OUTPUT_VARIABLE
        ECHO_ERROR_VARIABLE
        COMMAND_ECHO STDOUT
        TIMEOUT 300
    )

    if(NOT PYTEST_UTILS_RESULT EQUAL 0)
        message(FATAL_ERROR "ovphysx.utils Python tests failed (exit code: ${PYTEST_UTILS_RESULT})")
    endif()
    parse_pytest_junit("ovphysx.utils Python Tests" "${PYTEST_UTILS_RESULTS_XML}")
else()
    message(STATUS "No utils_tests/ directory found, skipping ovphysx.utils tests")
endif()

# ============================================================================
# Lifecycle tests (separate subprocess per test file)
#
# These tests exercise the PhysX create/destroy cycle. Carbonite and the
# embedded Python interpreter cannot be cleanly finalized and re-initialized
# in the same process, so each test file gets its own subprocess. Files run
# with bounded concurrency, by default min(build jobs, 4), which
# -DOVPHYSX_LIFECYCLE_JOBS=N overrides.
# ============================================================================

set(LIFECYCLE_TEST_DIR "${PYTHON_TEST_DIR}/lifecycle_tests")
if(EXISTS "${LIFECYCLE_TEST_DIR}")
    file(GLOB LIFECYCLE_TEST_FILES "${LIFECYCLE_TEST_DIR}/test_*.py")
    list(LENGTH LIFECYCLE_TEST_FILES LIFECYCLE_TEST_COUNT)
    ovphysx_compute_lifecycle_test_jobs(LIFECYCLE_JOBS)
    message(STATUS "")
    message(STATUS "Running lifecycle tests (${LIFECYCLE_TEST_COUNT} files, up to ${LIFECYCLE_JOBS} concurrent subprocesses)...")

    set(LIFECYCLE_PASS 0)
    set(LIFECYCLE_FAIL 0)
    set(LIFECYCLE_ALL_STDOUT "")
    set(LIFECYCLE_ALL_STDERR "")
    set(LIFECYCLE_JOB_DIR "${PYTEST_RESULTS_DIR}/lifecycle_jobs")
    file(MAKE_DIRECTORY "${LIFECYCLE_JOB_DIR}")
    set(LIFECYCLE_JOB_SCRIPTS "")

    if(WIN32)
        set(LIFECYCLE_PYTEST "${VENV_DIR}/Scripts/pytest.exe")
    else()
        set(LIFECYCLE_PYTEST "${VENV_DIR}/bin/pytest")
    endif()
    if(NOT EXISTS "${LIFECYCLE_PYTEST}")
        message(FATAL_ERROR
            "Lifecycle pytest not found at ${LIFECYCLE_PYTEST}. "
            "The main and CPU pytest invocations must populate the venv first.")
    endif()
    set(LIFECYCLE_ENV ${UV_ENV} "OVPHYSX_LIFECYCLE_SUBPROCESS=1")

    foreach(TEST_FILE ${LIFECYCLE_TEST_FILES})
        get_filename_component(TEST_NAME "${TEST_FILE}" NAME_WE)
        set(LIFECYCLE_XML "${PYTEST_RESULTS_DIR}/pytest_lifecycle_${TEST_NAME}.xml")
        file(REMOVE "${LIFECYCLE_XML}")

        if(WIN32)
            set(LIFECYCLE_JOB_SCRIPT "${LIFECYCLE_JOB_DIR}/${TEST_NAME}.bat")
        else()
            set(LIFECYCLE_JOB_SCRIPT "${LIFECYCLE_JOB_DIR}/${TEST_NAME}.sh")
        endif()
        file(REMOVE "${LIFECYCLE_JOB_SCRIPT}.log")
        file(REMOVE "${LIFECYCLE_JOB_SCRIPT}.err.log")

        # The venv pytest is used directly. Parallel "uv run" invocations race on the
        # shared project environment, and the main/CPU suites already synced the deps.
        set(LIFECYCLE_CMD
            ${CMAKE_COMMAND} -E env ${LIFECYCLE_ENV}
            "${LIFECYCLE_PYTEST}" "${TEST_FILE}"
            -p no:cacheprovider --junit-xml=${LIFECYCLE_XML} -v -s)
        ovphysx_write_job_script("${LIFECYCLE_JOB_SCRIPT}" "${PYTHON_TEST_DIR}" ${LIFECYCLE_CMD})
        list(APPEND LIFECYCLE_JOB_SCRIPTS "${LIFECYCLE_JOB_SCRIPT}")
    endforeach()

    ovphysx_run_job_scripts_parallel(
        "${LIFECYCLE_JOBS}" "${LIFECYCLE_JOB_SCRIPTS}"
        LIFECYCLE_FAILED_SCRIPTS LIFECYCLE_PARALLEL_OUTPUT)
    set(LIFECYCLE_ALL_STDOUT "${LIFECYCLE_PARALLEL_OUTPUT}")
    set(LIFECYCLE_ALL_STDERR "")

    foreach(TEST_FILE ${LIFECYCLE_TEST_FILES})
        get_filename_component(TEST_NAME "${TEST_FILE}" NAME_WE)
        if(WIN32)
            set(LIFECYCLE_JOB_SCRIPT "${LIFECYCLE_JOB_DIR}/${TEST_NAME}.bat")
        else()
            set(LIFECYCLE_JOB_SCRIPT "${LIFECYCLE_JOB_DIR}/${TEST_NAME}.sh")
        endif()
        list(FIND LIFECYCLE_FAILED_SCRIPTS "${LIFECYCLE_JOB_SCRIPT}" _failed_index)
        if(_failed_index GREATER -1)
            math(EXPR LIFECYCLE_FAIL "${LIFECYCLE_FAIL} + 1")
            message(WARNING "  ${TEST_NAME} FAILED")
        else()
            set(LIFECYCLE_XML "${PYTEST_RESULTS_DIR}/pytest_lifecycle_${TEST_NAME}.xml")
            if(NOT EXISTS "${LIFECYCLE_XML}")
                math(EXPR LIFECYCLE_FAIL "${LIFECYCLE_FAIL} + 1")
                message(WARNING "  ${TEST_NAME} FAILED (missing results XML: ${LIFECYCLE_XML})")
            else()
                math(EXPR LIFECYCLE_PASS "${LIFECYCLE_PASS} + 1")
                parse_pytest_junit("Lifecycle/${TEST_NAME}" "${LIFECYCLE_XML}")
            endif()
        endif()
    endforeach()

    message(STATUS "Lifecycle tests: ${LIFECYCLE_PASS} passed, ${LIFECYCLE_FAIL} failed out of ${LIFECYCLE_TEST_COUNT}")
    if(LIFECYCLE_FAIL GREATER 0)
        message(FATAL_ERROR "Lifecycle tests had ${LIFECYCLE_FAIL} failure(s)")
    endif()
else()
    message(STATUS "No lifecycle_tests/ directory found, skipping lifecycle tests")
endif()

# Check for simulation stage leaks in the output of all suites, including the lifecycle subprocesses.
set(PYTEST_ALL_OUTPUT "${PYTEST_STDOUT}${PYTEST_STDERR}${PYTEST_CPU_STDOUT}${PYTEST_CPU_STDERR}${LIFECYCLE_ALL_STDOUT}${LIFECYCLE_ALL_STDERR}")
if(PYTEST_ALL_OUTPUT MATCHES "outstanding SimStageWithHistory")
    string(REGEX MATCH "had ([0-9]+) outstanding" LEAK_MATCH "${PYTEST_ALL_OUTPUT}")
    if(CMAKE_MATCH_1)
        message(FATAL_ERROR "Simulation stage leak detected in Python tests: ${CMAKE_MATCH_1} outstanding SimStageWithHistory(s) at shutdown. All stages must be properly cleaned up.")
    else()
        message(FATAL_ERROR "Simulation stage leak detected in Python tests (see output). All stages must be properly cleaned up.")
    endif()
endif()

message(STATUS "Python unit tests: PASSED")

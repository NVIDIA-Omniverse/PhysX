# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

# ovphysx Python Tests
# Runs pytest-based Python unit tests against the INSTALLED SDK (_install/)
# Usage: cmake -P scripts/test_python_runtime.cmake
#
# NOTE: This script runs in an isolated process (via execute_process in validate_all.cmake).
# All test scripts run in isolated processes to avoid library conflicts between:
# - C++ tests that load Carbonite plugins with embedded Python
# - Python tests that pre-load 116+ native libraries with RTLD_GLOBAL

# Setup (self-contained)
cmake_minimum_required(VERSION 3.16)
get_filename_component(SCRIPT_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
include("${SCRIPT_DIR}/crossplatform_helpers.cmake")
include("${SCRIPT_DIR}/build_common.cmake")
include("${SCRIPT_DIR}/python_usd_test_overlay.cmake")

set(INSTALL_DIR "${PROJECT_ROOT}/_install")

# ovstage is not installed; consume the configured external package.
ovphysx_resolve_ovstage_paths()
set(OVSTAGE_PYTHON_DIR "${OVPHYSX_OVSTAGE_PYTHON_DIR}")
if(NOT OVSTAGE_PYTHON_DIR)
    message(FATAL_ERROR
        "The configured ovstage root '${OVPHYSX_OVSTAGE_ROOT}' carries no importable "
        "ovstage Python package; the Python runtime tests need one.")
endif()

# Python tests require a py-enabled USD monolith, while every other SDK test
# must see the installed py-less monolith. Run the existing test body in a
# child process so this parent can restore the install tree even when the child
# exits through message(FATAL_ERROR).
if(NOT OVPHYSX_PYTHON_RUNTIME_INNER)
    set(_PYUSD_DIR "")
    file(GLOB _PYUSD_CANDIDATES "${PROJECT_ROOT}/_build/target-deps/usd-py-tests/*")
    foreach(_candidate IN LISTS _PYUSD_CANDIDATES)
        if(EXISTS "${_candidate}/lib/python/pxr/__init__.py")
            set(_PYUSD_DIR "${_candidate}")
            break()
        endif()
    endforeach()
    if(NOT _PYUSD_DIR)
        message(FATAL_ERROR
            "py312 USD for python tests not found under _build/target-deps/usd-py-tests/*.\n"
            "Run scripts/fetch_deps.cmake (it pulls deps/usd-py-tests.packman.xml).")
    endif()

    if(WIN32)
        file(GLOB _PYUSD_MONOLITHS "${_PYUSD_DIR}/lib/*usd_ms.dll")
        set(_PYUSD_DESTINATIONS
            "${INSTALL_DIR}/plugins"
            "${OVSTAGE_PYTHON_DIR}/ovstage/bin/plugins")
    else()
        file(GLOB _PYUSD_MONOLITHS "${_PYUSD_DIR}/lib/lib*usd_ms.so")
        set(_PYUSD_DESTINATIONS
            "${INSTALL_DIR}/plugins"
            "${OVSTAGE_PYTHON_DIR}/ovstage/bin/plugins")
    endif()
    if(NOT _PYUSD_MONOLITHS)
        message(FATAL_ERROR "Python USD monolith not found under ${_PYUSD_DIR}/lib")
    endif()
    list(GET _PYUSD_MONOLITHS 0 _PYUSD_MONOLITH)

    set(_PYTHON_RUNTIME_CHILD_COMMAND
        "${CMAKE_COMMAND}"
        "-DBUILD_TYPE=${BUILD_TYPE}"
        "-DOVPHYSX_UV_COMMAND=${OVPHYSX_UV_COMMAND}"
        -DOVPHYSX_PYTHON_RUNTIME_INNER=ON
        "-DOVPHYSX_PYUSD_DIR=${_PYUSD_DIR}"
        -P "${CMAKE_CURRENT_LIST_FILE}")
    ovphysx_run_with_python_usd_overlay(
        MONOLITH "${_PYUSD_MONOLITH}"
        DESTINATIONS ${_PYUSD_DESTINATIONS}
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

# Use target-deps Python (3.12)
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

# Verify install exists
if(NOT EXISTS "${INSTALL_DIR}/plugins")
    message(FATAL_ERROR
        "Install directory not found at ${INSTALL_DIR}/plugins\n"
        "Run 'cmake -P scripts/install.cmake' first to create the SDK install tree.")
endif()

# Detect packman Python version
execute_process(
    COMMAND "${TARGET_PYTHON}" --version
    OUTPUT_VARIABLE PYTHON_VERSION_OUTPUT
    ERROR_VARIABLE PYTHON_VERSION_OUTPUT  # Some Python versions output to stderr
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_STRIP_TRAILING_WHITESPACE
)
string(REGEX MATCH "[0-9]+\\.[0-9]+\\.[0-9]+" PYTHON_VERSION "${PYTHON_VERSION_OUTPUT}")
if(NOT PYTHON_VERSION)
    set(PYTHON_VERSION "unknown")
endif()

# Check uv (needed for uv run pytest)
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

# Create venv using packman Python
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

# Clean stale editable metadata that can be left behind by previous runs
set(PYTHON_TEST_EGG_INFO "${PYTHON_TEST_DIR}/ovphysx_python_tests.egg-info")
if(EXISTS "${PYTHON_TEST_EGG_INFO}")
    file(REMOVE_RECURSE "${PYTHON_TEST_EGG_INFO}")
    if(EXISTS "${PYTHON_TEST_EGG_INFO}")
        message(FATAL_ERROR "Cannot remove stale egg-info dir: ${PYTHON_TEST_EGG_INFO}. Please fix permissions or delete it.")
    endif()
endif()

# Run pytest
set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "1")
# Store test results in _build to avoid polluting source tree
set(PYTEST_RESULTS_DIR "${PROJECT_ROOT}/_build/test_results")
file(MAKE_DIRECTORY "${PYTEST_RESULTS_DIR}")
set(PYTEST_RESULTS_XML "${PYTEST_RESULTS_DIR}/pytest_results.xml")

# Run pytest - cross-platform
set(UV_CACHE_DIR_PATH "${PROJECT_ROOT}/_build/.uv_cache")
file(MAKE_DIRECTORY "${UV_CACHE_DIR_PATH}")
set(OVSTAGE_WHEEL_DIR "${PROJECT_ROOT}/_build/target-deps/ovstage_wheel")
file(GLOB OVSTAGE_WHEELS "${OVSTAGE_WHEEL_DIR}/ovstage-*.whl")
if(NOT OVSTAGE_WHEELS)
    message(FATAL_ERROR "ovstage wheel not found in ${OVSTAGE_WHEEL_DIR}. Run the ovphysx build first.")
endif()
set(UV_ENV
    "UV_CACHE_DIR=${UV_CACHE_DIR_PATH}"
    "UV_NO_CONFIG=1"
    "UV_FIND_LINKS=${OVSTAGE_WHEEL_DIR}"
    "UV_HTTP_TIMEOUT=300"
    "UV_SKIP_WHEEL_FILENAME_CHECK=1"
)

if(NOT EXISTS "${OVSTAGE_PYTHON_DIR}/ovstage/__init__.py")
    message(FATAL_ERROR
        "ovstage Python package not found at ${OVSTAGE_PYTHON_DIR}/ovstage.\n"
        "Run scripts/fetch_deps.cmake so the ovstage release is extracted.")
endif()

# The parent process has overlaid the py-enabled monolith and passed its package
# root to this child. All native consumers now resolve one shared USD instance.
set(_PYUSD_DIR "${OVPHYSX_PYUSD_DIR}")
if(NOT _PYUSD_DIR OR NOT EXISTS "${_PYUSD_DIR}/lib/python/pxr/__init__.py")
    message(FATAL_ERROR "OVPHYSX_PYUSD_DIR is missing or invalid: ${_PYUSD_DIR}")
endif()

if(WIN32)
    set(OVSTAGE_RUNTIME_DIR "${OVPHYSX_OVSTAGE_RUNTIME_DIR}")
    set(OVSTAGE_RUNTIME_FILE "${OVSTAGE_RUNTIME_DIR}/ovstage.dll")
    set(_WINDOWS_PATH_SEGMENTS
        "${INSTALL_DIR}/bin"
        "${OVSTAGE_RUNTIME_DIR}"
        "${_PYUSD_DIR}/lib"
        "${INSTALL_DIR}/plugins"
        "${OVSTAGE_PYTHON_DIR}/ovstage/bin"
        "$ENV{PATH}"
    )
    list(JOIN _WINDOWS_PATH_SEGMENTS ";" _WINDOWS_PATH_VALUE)
    set(ENV{PATH} "${_WINDOWS_PATH_VALUE}")
    set(ENV{PYTHONPATH} "${_PYUSD_DIR}/lib/python;${OVSTAGE_PYTHON_DIR}")
    set(OVPHYSX_PYTHON_NATIVE_ENV
        "OVSTAGE_LIBRARY_PATH_HINT=${OVSTAGE_RUNTIME_DIR}"
    )
else()
    set(OVSTAGE_RUNTIME_DIR "${OVSTAGE_PYTHON_DIR}/ovstage/bin")
    set(OVSTAGE_RUNTIME_FILE "${OVSTAGE_RUNTIME_DIR}/libovstage.so")
    set(OVPHYSX_PYTHON_NATIVE_ENV
        "PYTHONPATH=${_PYUSD_DIR}/lib/python:${OVSTAGE_PYTHON_DIR}"
        "OVSTAGE_LIBRARY_PATH_HINT=${OVSTAGE_RUNTIME_DIR}"
        "LD_LIBRARY_PATH=${_PYUSD_DIR}/lib:${INSTALL_DIR}/lib:${INSTALL_DIR}/plugins:${OVSTAGE_RUNTIME_DIR}:$ENV{LD_LIBRARY_PATH}"
    )
endif()
if(NOT EXISTS "${OVSTAGE_RUNTIME_FILE}")
    message(FATAL_ERROR "ovstage runtime not found at ${OVSTAGE_RUNTIME_FILE}")
endif()
foreach(_OVPHYSX_ENV_ENTRY IN LISTS OVPHYSX_PYTHON_NATIVE_ENV)
    list(APPEND UV_ENV "${_OVPHYSX_ENV_ENTRY}")
endforeach()

# Exclude cpu_tests/ and lifecycle_tests/ here; they run in separate processes below.
message(STATUS "Running pytest...")
execute_process(
    COMMAND ${CMAKE_COMMAND} -E env ${UV_ENV} "${OVPHYSX_UV_COMMAND}" run pytest --ignore=cpu_tests --ignore=lifecycle_tests --junit-xml=${PYTEST_RESULTS_XML} -v -s
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

# Check exit code first - catch crashes even if XML was written
if(NOT PYTEST_RESULT EQUAL 0)
    message(FATAL_ERROR "Python tests failed (exit code: ${PYTEST_RESULT})")
endif()

parse_pytest_junit("Python Tests" "${PYTEST_RESULTS_XML}")

# ============================================================================
# CPU-mode tests (separate pytest invocation)
#
# Carbonite/PhysX device mode is a process-global singleton: the first PhysX()
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
# Lifecycle tests (separate subprocess per test file)
#
# These tests exercise the PhysX create/release cycle. Carbonite and the
# embedded Python interpreter cannot be cleanly finalized and re-initialized
# in the same process, so each test file gets its own subprocess.
# ============================================================================

set(LIFECYCLE_TEST_DIR "${PYTHON_TEST_DIR}/lifecycle_tests")
if(EXISTS "${LIFECYCLE_TEST_DIR}")
    file(GLOB LIFECYCLE_TEST_FILES "${LIFECYCLE_TEST_DIR}/test_*.py")
    list(LENGTH LIFECYCLE_TEST_FILES LIFECYCLE_TEST_COUNT)
    message(STATUS "")
    message(STATUS "Running lifecycle tests (${LIFECYCLE_TEST_COUNT} files, each in its own process)...")

    set(LIFECYCLE_PASS 0)
    set(LIFECYCLE_FAIL 0)
    set(LIFECYCLE_ALL_STDOUT "")
    set(LIFECYCLE_ALL_STDERR "")

    foreach(TEST_FILE ${LIFECYCLE_TEST_FILES})
        get_filename_component(TEST_NAME "${TEST_FILE}" NAME_WE)
        set(LIFECYCLE_XML "${PYTEST_RESULTS_DIR}/pytest_lifecycle_${TEST_NAME}.xml")
        file(REMOVE "${LIFECYCLE_XML}")

        message(STATUS "  Running ${TEST_NAME}...")
        execute_process(
            COMMAND ${CMAKE_COMMAND} -E env ${UV_ENV} "${OVPHYSX_UV_COMMAND}" run pytest "${TEST_FILE}" --junit-xml=${LIFECYCLE_XML} -v -s
            WORKING_DIRECTORY "${PYTHON_TEST_DIR}"
            RESULT_VARIABLE LIFECYCLE_RESULT
            OUTPUT_VARIABLE LIFECYCLE_STDOUT
            ERROR_VARIABLE LIFECYCLE_STDERR
            ECHO_OUTPUT_VARIABLE
            ECHO_ERROR_VARIABLE
            TIMEOUT 120
        )

        string(APPEND LIFECYCLE_ALL_STDOUT "${LIFECYCLE_STDOUT}")
        string(APPEND LIFECYCLE_ALL_STDERR "${LIFECYCLE_STDERR}")

        if(NOT LIFECYCLE_RESULT STREQUAL "0")
            math(EXPR LIFECYCLE_FAIL "${LIFECYCLE_FAIL} + 1")
            message(WARNING "  ${TEST_NAME} FAILED (exit code: ${LIFECYCLE_RESULT})")
        else()
            math(EXPR LIFECYCLE_PASS "${LIFECYCLE_PASS} + 1")
            if(EXISTS "${LIFECYCLE_XML}")
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

# Check for simulation stage leaks in Python test output (all suites including lifecycle subprocesses)
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

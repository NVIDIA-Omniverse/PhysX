# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

# ovphysx Python Wheel Smoke Tests
# Validates the installed wheel via `python -m ovphysx` across Python versions.
# Usage: cmake -P scripts/test_python_wheel.cmake
#
# Prerequisites: cmake -P scripts/build_wheel.cmake

cmake_minimum_required(VERSION 3.16)

get_filename_component(SCRIPT_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
get_filename_component(PROJECT_ROOT "${SCRIPT_DIR}/.." ABSOLUTE)
include("${SCRIPT_DIR}/build_common.cmake")

message(STATUS "")
message(STATUS "=== Python Wheel Smoke Tests ===")

# Verify uv is available
execute_process(
    COMMAND "${OVPHYSX_UV_COMMAND}" --version
    OUTPUT_VARIABLE UV_VERSION
    ERROR_QUIET
    RESULT_VARIABLE UV_CHECK
)
if(NOT UV_CHECK EQUAL 0)
    message(FATAL_ERROR "uv not found. Install it via: https://docs.astral.sh/uv/getting-started/installation/")
endif()
string(STRIP "${UV_VERSION}" UV_VERSION)
message(STATUS "Found uv: ${UV_VERSION}")

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

# Clean environment - wheel tests should be self-contained
unset(ENV{LD_LIBRARY_PATH})
unset(ENV{OVPHYSX_LIB})
unset(ENV{OVSTAGE_LIBRARY_PATH_HINT})
unset(ENV{VIRTUAL_ENV})

# Check if SDK wheel exists
file(GLOB WHEEL_FILES "${PROJECT_ROOT}/_dist/ovphysx-*.whl")
if(NOT WHEEL_FILES)
    message(FATAL_ERROR
        "SDK wheel not found in ${PROJECT_ROOT}/_dist/.\n"
        "Run: cmake -P scripts/build_wheel.cmake")
endif()
list(LENGTH WHEEL_FILES _WHEEL_COUNT)
if(_WHEEL_COUNT GREATER 1)
    message(FATAL_ERROR
        "Multiple wheels found in ${PROJECT_ROOT}/_dist/ (${_WHEEL_COUNT}):\n"
        "  ${WHEEL_FILES}\n"
        "Clean _dist/ and rebuild to ensure the smoke test validates the correct artifact.")
endif()
list(GET WHEEL_FILES 0 SDK_WHEEL_PATH)

set(BUNDLED_OVERRIDE_SMOKE
    "${PROJECT_ROOT}/tests/python_tests/wheel_bundled_override_smoke.py")
if(NOT EXISTS "${BUNDLED_OVERRIDE_SMOKE}")
    message(FATAL_ERROR "Bundled override smoke test not found: ${BUNDLED_OVERRIDE_SMOKE}")
endif()

set(PYTHON_VERSIONS
    "3.10"
    "3.11"
    "3.12"
    "3.13"
)

set(VENV_BASE "${PROJECT_ROOT}/_build/.venv_wheel_smoke")

set(TESTS_PASSED 0)
set(TESTS_FAILED 0)

foreach(PY_VER ${PYTHON_VERSIONS})
    message(STATUS "")
    message(STATUS "--- Wheel smoke test (Python ${PY_VER}) ---")

    ensure_uv_managed_python(${PY_VER} UV_PYTHON_PATH)

    set(_VENV_DIR "${VENV_BASE}_${PY_VER}")
    message(STATUS "Creating fresh venv at ${_VENV_DIR}...")
    file(REMOVE_RECURSE "${_VENV_DIR}")
    execute_process(
        COMMAND "${OVPHYSX_UV_COMMAND}" venv --python "${UV_PYTHON_PATH}" "${_VENV_DIR}"
        RESULT_VARIABLE _VENV_RESULT
        ERROR_VARIABLE _VENV_ERROR
    )
    if(NOT _VENV_RESULT STREQUAL "0")
        message(FATAL_ERROR "Failed to create venv for Python ${PY_VER}: ${_VENV_ERROR}")
    endif()

    message(STATUS "Installing wheel: ${SDK_WHEEL_PATH}")
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E env ${UV_ENV}
            "${OVPHYSX_UV_COMMAND}" pip install --python ${_VENV_DIR} ${SDK_WHEEL_PATH}
        RESULT_VARIABLE _INSTALL_RESULT
    )
    if(NOT _INSTALL_RESULT STREQUAL "0")
        message(FATAL_ERROR "Failed to install SDK wheel for Python ${PY_VER}")
    endif()

    # Verify `python -m ovphysx` entrypoint works
    message(STATUS "Running python -m ovphysx...")
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E env ${UV_ENV}
            "${OVPHYSX_UV_COMMAND}" run --python ${_VENV_DIR} --no-sync -- python -m ovphysx
        RESULT_VARIABLE _MODULE_RESULT
        OUTPUT_VARIABLE _MODULE_OUTPUT
        ERROR_VARIABLE _MODULE_OUTPUT
        ECHO_OUTPUT_VARIABLE
        ECHO_ERROR_VARIABLE
    )

    if(NOT _MODULE_RESULT STREQUAL "0")
        message(STATUS "Module output: ${_MODULE_OUTPUT}")
        message(FATAL_ERROR "python -m ovphysx failed for Python ${PY_VER} (exit code: ${_MODULE_RESULT})")
    else()
        message(STATUS "  [PASS] python -m ovphysx (Python ${PY_VER})")
    endif()

    message(STATUS "Running bundled OVPHYSX_LIB override smoke test...")
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E env ${UV_ENV}
            "${OVPHYSX_UV_COMMAND}" run --python ${_VENV_DIR} --no-sync --
            python "${BUNDLED_OVERRIDE_SMOKE}"
        RESULT_VARIABLE _OVERRIDE_RESULT
        OUTPUT_VARIABLE _OVERRIDE_OUTPUT
        ERROR_VARIABLE _OVERRIDE_OUTPUT
        ECHO_OUTPUT_VARIABLE
        ECHO_ERROR_VARIABLE
    )

    if(NOT _OVERRIDE_RESULT STREQUAL "0")
        message(STATUS "Bundled override output: ${_OVERRIDE_OUTPUT}")
        message(FATAL_ERROR
            "Bundled OVPHYSX_LIB override failed for Python ${PY_VER} "
            "(exit code: ${_OVERRIDE_RESULT})")
    else()
        message(STATUS "  [PASS] bundled OVPHYSX_LIB override (Python ${PY_VER})")
        math(EXPR TESTS_PASSED "${TESTS_PASSED} + 1")
    endif()
endforeach()

message(STATUS "")
message(STATUS "Wheel smoke tests: ${TESTS_PASSED} Python versions passed")
message(STATUS "Python wheel smoke tests: PASSED")

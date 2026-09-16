# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PACKAGING-TESTDEPS-001
# @covers AC-5 AC-6

# CI validation script.
cmake_minimum_required(VERSION 3.16)

get_filename_component(SCRIPT_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
get_filename_component(PROJECT_ROOT "${SCRIPT_DIR}/.." ABSOLUTE)
include("${SCRIPT_DIR}/crossplatform_helpers.cmake")

message(STATUS "Validating formatting (ovphysx)")
execute_process(
    COMMAND "${PROJECT_ROOT}/repo${SCRIPT_SUFFIX}" format --legal-only --verify
    WORKING_DIRECTORY "${PROJECT_ROOT}"
    RESULT_VARIABLE FORMAT_RESULT
)
if(NOT FORMAT_RESULT STREQUAL "0")
    message(FATAL_ERROR "ovphysx formatting validation failed (exit code: ${FORMAT_RESULT})")
endif()
message(STATUS "  [OK] ovphysx formatting validation passed")

# ---------------------------------------------------------------------------
# uv.lock validation
#
# ovstage is pinned by version but must resolve from the build-staged local wheels
# each lock names relative to its own directory, not from PyPI URLs. An accidental
# `uv run` without find-links/--locked can rewrite a lock with no version bump.
# Locks the public source drop leaves out are skipped there.
# ---------------------------------------------------------------------------
message(STATUS "Validating checked-in uv.lock files (ovstage source)")
find_package(Python3 COMPONENTS Interpreter REQUIRED)
execute_process(
    COMMAND "${Python3_EXECUTABLE}" "${SCRIPT_DIR}/validate_python_test_uv_lock.py"
    WORKING_DIRECTORY "${PROJECT_ROOT}"
    RESULT_VARIABLE UV_LOCK_VALIDATE_RESULT
    ERROR_VARIABLE UV_LOCK_VALIDATE_ERROR
    ERROR_STRIP_TRAILING_WHITESPACE
    OUTPUT_VARIABLE UV_LOCK_VALIDATE_OUTPUT
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT UV_LOCK_VALIDATE_RESULT STREQUAL "0")
    message(FATAL_ERROR
        "uv.lock validation failed (exit ${UV_LOCK_VALIDATE_RESULT}): "
        "${UV_LOCK_VALIDATE_ERROR}\n${UV_LOCK_VALIDATE_OUTPUT}")
endif()
message(STATUS "  [OK] ${UV_LOCK_VALIDATE_OUTPUT}")

message(STATUS "")
message(STATUS "Validating Python type stubs (pyright)")
execute_process(
    COMMAND ${CMAKE_COMMAND} -P "${SCRIPT_DIR}/test_pyright.cmake"
    WORKING_DIRECTORY "${PROJECT_ROOT}"
    RESULT_VARIABLE PYRIGHT_RESULT
)
if(NOT PYRIGHT_RESULT EQUAL 0)
    message(FATAL_ERROR "pyright validation failed (exit code: ${PYRIGHT_RESULT})")
endif()
message(STATUS "  [OK] pyright validation passed")

message(STATUS "Validating CMake version requirement (ovphysx)")
execute_process(
    COMMAND "${CMAKE_COMMAND}"
            "-DVERSION_MODULE=${PROJECT_ROOT}/cmake/RequireCMakeVersion.cmake"
            -P "${PROJECT_ROOT}/tests/cmake/test_require_cmake_version.cmake"
    RESULT_VARIABLE REQUIRE_CMAKE_VERSION_RESULT
)
if(NOT REQUIRE_CMAKE_VERSION_RESULT STREQUAL "0")
    message(FATAL_ERROR "ovphysx CMake version requirement tests failed (exit code: ${REQUIRE_CMAKE_VERSION_RESULT})")
endif()
message(STATUS "  [OK] ovphysx CMake version requirement tests passed")

message(STATUS "Validating host path helpers (ovphysx)")
execute_process(
    COMMAND "${CMAKE_COMMAND}"
            "-DTEST_ROOT=${PROJECT_ROOT}/_build/tests/host_path_utils"
            "-DPATH_HELPER=${SCRIPT_DIR}/host_path_utils.cmake"
            -P "${PROJECT_ROOT}/tests/cmake/test_host_path_utils.cmake"
    RESULT_VARIABLE HOST_PATH_UTILS_RESULT
)
if(NOT HOST_PATH_UTILS_RESULT STREQUAL "0")
    message(FATAL_ERROR "ovphysx host path helper tests failed (exit code: ${HOST_PATH_UTILS_RESULT})")
endif()
message(STATUS "  [OK] ovphysx host path helper tests passed")

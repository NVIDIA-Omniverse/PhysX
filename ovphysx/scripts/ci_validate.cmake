# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

# CI Validation Script
cmake_minimum_required(VERSION 3.16)

# Get script directory and project root
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

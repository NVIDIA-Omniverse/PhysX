# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-BENCHMARK-001
# @covers AC-1 AC-4

cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED HELPER)
    message(FATAL_ERROR "HELPER must name benchmark_driver_common.cmake")
endif()
if(NOT DEFINED MODE)
    message(FATAL_ERROR "MODE must be configure or check-report")
endif()

include("${HELPER}")

if(MODE STREQUAL "configure")
    ovphysx_benchmark_driver_configure(
        "${RUN_GPU}"
        "${RUN_CPU}"
        "${RUN_CPU_ST}"
        "${HIDDEN}"
        "${EXPECT_ROWS}"
    )
    set(_ARGS "--base")
    ovphysx_benchmark_driver_append_hidden(_ARGS)
    list(JOIN _ARGS "|" _ARGS_TEXT)
    message(STATUS "ENABLED_PASSES=${BENCHMARK_DRIVER_ENABLED_PASSES}")
    message(STATUS "HIDDEN=${BENCHMARK_DRIVER_HIDDEN}")
    message(STATUS "EXPECT_ROWS=${BENCHMARK_DRIVER_EXPECT_ROWS}")
    message(STATUS "ARGS=${_ARGS_TEXT}")
elseif(MODE STREQUAL "check-report")
    if(NOT DEFINED REPORT)
        message(FATAL_ERROR "REPORT must name a benchmark report")
    endif()
    ovphysx_benchmark_driver_require_rows("${REPORT}" "${EXPECTED_ROWS}" "probe")
elseif(MODE STREQUAL "write-status")
    if(NOT DEFINED STATUS_FILE)
        message(FATAL_ERROR "STATUS_FILE must name the output sidecar")
    endif()
    ovphysx_benchmark_driver_write_status("${STATUS_FILE}" "${RETURN_CODE}")
else()
    message(FATAL_ERROR "unsupported MODE: ${MODE}")
endif()

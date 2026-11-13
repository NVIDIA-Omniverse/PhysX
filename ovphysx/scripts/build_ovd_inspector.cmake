# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# Build a separate SDK tool for integration tests; it is not a wheel dependency.
cmake_minimum_required(VERSION 3.21)
include("${CMAKE_CURRENT_LIST_DIR}/build_common.cmake")

get_filename_component(_physx_root "${PROJECT_ROOT}/../physx" ABSOLUTE)
if(NOT EXISTS "${_physx_root}/pvddom/CMakeLists.txt")
    message(FATAL_ERROR "Inspector sources are absent. Build ovd-inspect from the PhysX SDK and set OVPHYSX_OVD_INSPECTOR to its installed executable for validation.")
endif()
set(_tool_build "${BUILD_PATH}/ovd-inspector-build")
set(_tool_install "${BUILD_PATH}/ovd-inspector-install")
set(_configure_args -S "${_physx_root}/pvddom" -B "${_tool_build}"
    "-DPVDRUNTIME_ROOT=${_physx_root}/pvdruntime" -DPVDDOM_BUILD_INSPECTION=ON -DPVDDOM_BUILD_TOOLS=ON
    -DPVDDOM_BUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release)
ovphysx_pin_packaged_host_toolchain(_configure_args)
execute_process(COMMAND "${CMAKE_COMMAND}" ${_configure_args} RESULT_VARIABLE _result)
if(NOT "${_result}" STREQUAL "0")
    message(FATAL_ERROR "Inspector configure failed: ${_result}")
endif()
ovphysx_compute_build_jobs(_jobs)
execute_process(COMMAND "${CMAKE_COMMAND}" --build "${_tool_build}" --config Release --parallel "${_jobs}"
    RESULT_VARIABLE _result)
if(NOT "${_result}" STREQUAL "0")
    message(FATAL_ERROR "Inspector build failed: ${_result}")
endif()
execute_process(COMMAND "${CMAKE_COMMAND}" --install "${_tool_build}" --config Release --prefix "${_tool_install}"
    RESULT_VARIABLE _result)
if(NOT "${_result}" STREQUAL "0")
    message(FATAL_ERROR "Inspector install failed: ${_result}")
endif()

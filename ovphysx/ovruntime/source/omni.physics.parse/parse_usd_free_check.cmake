# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# CI gate: verify no USD/pxr includes leak into omni.physics.parse's public headers (PARSE_INCLUDE_DIR)
# or its own .cpp/.h/.hpp sources, including private (non-installed) headers that live alongside
# the .cpp files (PARSE_SOURCE_DIR). The `omni/physics/usd/` subdirectory is the USD backend's
# public surface and is excluded; PARSE_SOURCE_DIR's `tests/` subtree is excluded (tests may use
# pxr freely).
#
# Invoked as: cmake -DPARSE_INCLUDE_DIR=<dir> [-DPARSE_SOURCE_DIR=<dir>] -P parse_usd_free_check.cmake

if(NOT DEFINED PARSE_INCLUDE_DIR)
    message(FATAL_ERROR "PARSE_INCLUDE_DIR must be defined")
endif()

file(GLOB_RECURSE _headers
    LIST_DIRECTORIES false
    "${PARSE_INCLUDE_DIR}/*.h"
    "${PARSE_INCLUDE_DIR}/*.hpp"
)

set(_files "${_headers}")
if(DEFINED PARSE_SOURCE_DIR)
    file(GLOB_RECURSE _sources
        LIST_DIRECTORIES false
        "${PARSE_SOURCE_DIR}/*.cpp"
        "${PARSE_SOURCE_DIR}/*.h"
        "${PARSE_SOURCE_DIR}/*.hpp"
    )
    list(APPEND _files "${_sources}")
endif()

set(_offenders "")
foreach(_f IN LISTS _files)
    # Skip the USD backend's public surface and this module's own tests.
    string(FIND "${_f}" "/omni/physics/usd/" _usd_idx)
    string(FIND "${_f}" "/tests/" _tests_idx)
    if(NOT _usd_idx EQUAL -1 OR NOT _tests_idx EQUAL -1)
        continue()
    endif()
    file(READ "${_f}" _content)
    if(_content MATCHES "pxr/" OR _content MATCHES "PXR_NS::")
        list(APPEND _offenders "${_f}")
    endif()
endforeach()

if(_offenders)
    message("ERROR: USD includes found in omni.physics.parse core headers/sources:")
    foreach(_f IN LISTS _offenders)
        message("  ${_f}")
    endforeach()
    message(FATAL_ERROR "USD includes leaked into core parse headers/sources")
endif()

message(STATUS "OK: no USD includes in core parse headers/sources")

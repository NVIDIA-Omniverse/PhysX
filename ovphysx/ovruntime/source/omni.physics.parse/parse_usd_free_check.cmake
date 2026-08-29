# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# CI gate: verify no USD/pxr includes leak into core omni.physics.parse public headers.
# The `omni/physics/usd/` subdirectory is the USD backend's public surface and is excluded.
#
# Invoked as: cmake -DPARSE_INCLUDE_DIR=<dir> -P parse_usd_free_check.cmake

if(NOT DEFINED PARSE_INCLUDE_DIR)
    message(FATAL_ERROR "PARSE_INCLUDE_DIR must be defined")
endif()

file(GLOB_RECURSE _headers
    LIST_DIRECTORIES false
    "${PARSE_INCLUDE_DIR}/*.h"
    "${PARSE_INCLUDE_DIR}/*.hpp"
)

set(_offenders "")
foreach(_h IN LISTS _headers)
    # Skip the USD backend's public surface.
    string(FIND "${_h}" "/omni/physics/usd/" _idx)
    if(NOT _idx EQUAL -1)
        continue()
    endif()
    file(READ "${_h}" _content)
    if(_content MATCHES "pxr/" OR _content MATCHES "PXR_NS::")
        list(APPEND _offenders "${_h}")
    endif()
endforeach()

if(_offenders)
    message("ERROR: USD includes found in omni.physics.parse core public headers:")
    foreach(_f IN LISTS _offenders)
        message("  ${_f}")
    endforeach()
    message(FATAL_ERROR "USD includes leaked into core parse headers")
endif()

message(STATUS "OK: no USD includes in core parse headers")

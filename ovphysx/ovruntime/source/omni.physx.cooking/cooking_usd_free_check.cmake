# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# CI gate: verify no USD/pxr includes leak into omni.physx.cooking's own plugins/ tree (the
# production cooking library). tests/ lives in a sibling directory and is out of scope for this
# scan -- the unit tests use pxr deliberately to build USD scenes and hand cooking a mesh view.
#
# Invoked as: cmake -DCOOKING_PLUGINS_DIR=<dir> -P cooking_usd_free_check.cmake

if(NOT DEFINED COOKING_PLUGINS_DIR)
    message(FATAL_ERROR "COOKING_PLUGINS_DIR must be defined")
endif()

file(GLOB_RECURSE _files
    LIST_DIRECTORIES false
    "${COOKING_PLUGINS_DIR}/*.cpp"
    "${COOKING_PLUGINS_DIR}/*.h"
    "${COOKING_PLUGINS_DIR}/*.hpp"
    "${COOKING_PLUGINS_DIR}/*.inl"
)

set(_offenders "")
foreach(_f IN LISTS _files)
    file(READ "${_f}" _content)
    if(_content MATCHES "pxr/" OR _content MATCHES "PXR_NS::")
        list(APPEND _offenders "${_f}")
    endif()
endforeach()

if(_offenders)
    message("ERROR: USD includes found in omni.physx.cooking plugins/:")
    foreach(_f IN LISTS _offenders)
        message("  ${_f}")
    endforeach()
    message(FATAL_ERROR "USD includes leaked into omni.physx.cooking plugins/")
endif()

message(STATUS "OK: no USD includes in omni.physx.cooking plugins/")

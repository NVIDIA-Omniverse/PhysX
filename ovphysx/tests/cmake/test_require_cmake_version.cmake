# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.16)

# @implements TEST-BUILD-CMAKE-001
# @covers AC-1 AC-2 AC-4

if(NOT DEFINED VERSION_MODULE)
    message(FATAL_ERROR "VERSION_MODULE is required")
endif()

include("${VERSION_MODULE}")

# Assert the declared floor, so moving it is a deliberate edit here too.
if(NOT OVPHYSX_WINDOWS_MIN_CMAKE_VERSION STREQUAL "4.1.0")
    message(FATAL_ERROR
        "Unexpected Windows CMake floor: ${OVPHYSX_WINDOWS_MIN_CMAKE_VERSION} (expected 4.1.0)")
endif()

# --- Windows hosts below the floor are rejected ---
# 4.0.9 is the interesting one. It is newer than the project's 3.16 baseline, so
# only the Windows floor can reject it.
foreach(_too_old "3.16.0" "3.31.0" "4.0.0" "4.0.9")
    ovphysx_check_cmake_version(1 "${_too_old}" _error)
    if(NOT _error)
        message(FATAL_ERROR "Windows host with CMake ${_too_old} should have been rejected")
    endif()
    # The message has to name both numbers, otherwise it does not tell the user
    # what to install or what they have.
    if(NOT _error MATCHES "4\\.1\\.0")
        message(FATAL_ERROR "Error text omits the required version: ${_error}")
    endif()
    if(NOT _error MATCHES "${_too_old}")
        message(FATAL_ERROR "Error text omits the found version: ${_error}")
    endif()
endforeach()

# --- Windows hosts at or above the floor are accepted ---
# 4.1.0 itself must pass. The requirement is "at least", not "newer than".
foreach(_ok "4.1.0" "4.1.1" "4.2.0" "5.0.0")
    ovphysx_check_cmake_version(1 "${_ok}" _error)
    if(_error)
        message(FATAL_ERROR "Windows host with CMake ${_ok} should have been accepted: ${_error}")
    endif()
endforeach()

# --- Non-Windows hosts keep the 3.16 baseline ---
foreach(_old "3.16.0" "4.0.9")
    ovphysx_check_cmake_version("" "${_old}" _error)
    if(_error)
        message(FATAL_ERROR "Non-Windows host with CMake ${_old} should have been accepted: ${_error}")
    endif()
endforeach()

# --- The enforcing wrapper exists and passes on this host ---
# The test only runs on CMake versions that already satisfy the floor (it is
# running one), so this must not abort.
if(NOT COMMAND ovphysx_require_cmake_version)
    message(FATAL_ERROR "ovphysx_require_cmake_version is not defined")
endif()
ovphysx_require_cmake_version()

message(STATUS "test_require_cmake_version: PASS")

# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-BUILD-CMAKE-001
# @covers AC-1 AC-2 AC-4
#
# Windows-only CMake floor: an extra requirement on top of the project's 3.16
# baseline, not a bump of it.
#
# Older CMake strips the backslashes out of the MSVC host-compiler path it hands
# to nvcc as `-ccbin`, so the CUDA configure step fails. 4.1.0 is the first
# release that stops doing it.
#
# Call ovphysx_require_cmake_version() from every entry point. The script-mode
# ones run as `cmake -P` and never read CMakeLists.txt.

include_guard(GLOBAL)

set(OVPHYSX_WINDOWS_MIN_CMAKE_VERSION "4.1.0")

# Kept separate from the enforcement so the test can drive both sides of the
# threshold without an old CMake or a Windows host. Sets <out_var> to the error
# text, or "" when the version is acceptable.
function(ovphysx_check_cmake_version host_is_windows version out_var)
    set(${out_var} "" PARENT_SCOPE)
    if(NOT host_is_windows)
        return()
    endif()
    if(version VERSION_LESS "${OVPHYSX_WINDOWS_MIN_CMAKE_VERSION}")
        set(${out_var}
            "ovphysx requires CMake ${OVPHYSX_WINDOWS_MIN_CMAKE_VERSION} or newer on Windows (found ${version}). Older CMake mangles the MSVC host-compiler path passed to nvcc, so the CUDA configure step fails. Upgrade CMake and re-run."
            PARENT_SCOPE)
    endif()
endfunction()

# Enforce the floor for the running CMake.
function(ovphysx_require_cmake_version)
    ovphysx_check_cmake_version("${CMAKE_HOST_WIN32}" "${CMAKE_VERSION}" _error)
    if(_error)
        message(FATAL_ERROR "${_error}")
    endif()
endfunction()

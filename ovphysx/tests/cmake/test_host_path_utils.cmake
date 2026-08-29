# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED TEST_ROOT)
    message(FATAL_ERROR "TEST_ROOT is required")
endif()
if(NOT DEFINED PATH_HELPER)
    message(FATAL_ERROR "PATH_HELPER is required")
endif()

include("${PATH_HELPER}")

set(_spaced_dir "${TEST_ROOT}/dir with spaces/nested")
file(REMOVE_RECURSE "${TEST_ROOT}")
file(MAKE_DIRECTORY "${_spaced_dir}")

# --- ovphysx_normalize_host_path: backslashes become forward slashes ---
# Only Windows has a separate native spelling. On POSIX file(TO_NATIVE_PATH)
# shell-escapes spaces as "\ ", which normalization would then read as
# separators, so feed the cmake-style path straight through.
if(CMAKE_HOST_WIN32)
    file(TO_NATIVE_PATH "${_spaced_dir}" _native)
else()
    set(_native "${_spaced_dir}")
endif()
ovphysx_normalize_host_path("${_native}" _normalized)
if(_normalized MATCHES "\\\\")
    message(FATAL_ERROR "Backslash survived normalization: ${_normalized}")
endif()
if(NOT EXISTS "${_normalized}")
    message(FATAL_ERROR "Normalized path does not resolve: ${_normalized}")
endif()

# --- ovphysx_space_free_host_path: forward-slashed, space-free, still resolves ---
ovphysx_space_free_host_path("${_native}" _space_free)
if(_space_free MATCHES "\\\\")
    message(FATAL_ERROR "Backslash survived space-free conversion: ${_space_free}")
endif()
if(NOT EXISTS "${_space_free}")
    message(FATAL_ERROR "Space-free path does not resolve: ${_space_free}")
endif()

# The helper must return the spelling Windows resolves, which is what CMake will
# emit -- not merely something without spaces. Resolve it independently so that a
# helper which only normalizes cannot pass by looking like the 8.3-disabled
# fallback.
if(CMAKE_HOST_WIN32)
    execute_process(
        COMMAND cmd /c for %I in ("${_native}") do @echo %~sI
        OUTPUT_VARIABLE _oracle
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE _oracle_result
        ERROR_QUIET
    )
    if(_oracle_result EQUAL 0 AND _oracle)
        ovphysx_normalize_host_path("${_oracle}" _oracle_normalized)
        if(NOT _space_free STREQUAL _oracle_normalized)
            message(FATAL_ERROR
                "Helper did not return the resolved short spelling:\n"
                "  got:      ${_space_free}\n"
                "  expected: ${_oracle_normalized}")
        endif()
    else()
        message(STATUS "No 8.3 short name on this volume; long-form fallback checked")
    endif()
else()
    # Spaces are legal everywhere else, so the helper only normalizes.
    if(NOT _space_free STREQUAL _normalized)
        message(FATAL_ERROR "Non-Windows host should return the normalized path")
    endif()
endif()

# --- An already space-free path must be returned unchanged ---
set(_plain_dir "${TEST_ROOT}/plain")
file(MAKE_DIRECTORY "${_plain_dir}")
ovphysx_space_free_host_path("${_plain_dir}" _plain_result)
if(NOT _plain_result STREQUAL _plain_dir)
    message(FATAL_ERROR "Space-free input was altered: ${_plain_dir} -> ${_plain_result}")
endif()

file(REMOVE_RECURSE "${TEST_ROOT}")
message(STATUS "test_host_path_utils: PASS")

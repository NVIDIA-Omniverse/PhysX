# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PACKAGING-BUILDNUM-001
# @covers AC-1 AC-2 AC-4 AC-6

# Print the build-numbered artifact version for non-CMake packaging consumers.
# Tag the value so consumers can ignore STATUS messages on CMake 3.16 too.
include("${CMAKE_CURRENT_LIST_DIR}/crossplatform_helpers.cmake")
if(NOT DEFINED VERSION_FILE)
    set(VERSION_FILE "${CMAKE_CURRENT_LIST_DIR}/../VERSION")
endif()
file(READ "${VERSION_FILE}" VERSION)
string(STRIP "${VERSION}" VERSION)
if(VERSION STREQUAL "")
    message(FATAL_ERROR "VERSION file is empty: ${VERSION_FILE}")
endif()
apply_build_number("${VERSION}" VERSION)
execute_process(
    COMMAND "${CMAKE_COMMAND}" -E echo "OVPHYSX_ARTIFACT_VERSION=${VERSION}"
    RESULT_VARIABLE _echo_result
)
if(NOT "${_echo_result}" STREQUAL "0")
    message(FATAL_ERROR "Could not print the artifact version: ${_echo_result}")
endif()

# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PACKAGING-OVSTAGEVERSION-001
# @covers AC-1 AC-3

# Read the same release pin used by the source fetcher and Python dependency.
# OVStage's native CMake package version contains only major.minor.patch.
function(ovphysx_read_ovstage_version fetch_script out_version out_cmake_version)
    if(NOT EXISTS "${fetch_script}")
        message(FATAL_ERROR "OVStage version source is missing: ${fetch_script}")
    endif()
    file(STRINGS "${fetch_script}" _ovstage_pins REGEX "^OVSTAGE_VERSION[ \t]*=")
    list(LENGTH _ovstage_pins _ovstage_pin_count)
    if(NOT _ovstage_pin_count EQUAL 1)
        message(FATAL_ERROR "Expected one OVSTAGE_VERSION assignment in ${fetch_script}")
    endif()
    if(NOT _ovstage_pins MATCHES
        "^OVSTAGE_VERSION[ \t]*=[ \t]*\"(([0-9]+[.][0-9]+[.][0-9]+)([.][0-9]+([.][0-9A-Za-z]+)?)?)\"[ \t]*$")
        message(FATAL_ERROR "Invalid OVSTAGE_VERSION in ${fetch_script}")
    endif()
    set(${out_version} "${CMAKE_MATCH_1}" PARENT_SCOPE)
    set(${out_cmake_version} "${CMAKE_MATCH_2}" PARENT_SCOPE)
endfunction()

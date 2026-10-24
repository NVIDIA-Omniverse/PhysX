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

# The source override must export the API version it actually compiled against.
# It is not a downloadable release identity and must never bootstrap the stock pin.
function(ovphysx_read_selected_ovstage_version include_dirs out_version out_header)
    set(headers)
    foreach(directory IN LISTS include_dirs)
        set(header "${directory}/ovstage/ovstage_api/ovstage_api_types.h")
        if(EXISTS "${header}")
            get_filename_component(header "${header}" REALPATH)
            list(APPEND headers "${header}")
        endif()
    endforeach()
    list(REMOVE_DUPLICATES headers)
    list(LENGTH headers count)
    if(NOT count EQUAL 1)
        message(FATAL_ERROR "Expected one selected OVStage API header; found ${count}")
    endif()
    list(GET headers 0 header)
    set(parts)
    foreach(part IN ITEMS MAJOR MINOR PATCH)
        file(STRINGS "${header}" lines REGEX "^#define OVSTAGE_VERSION_${part} [0-9]+$")
        list(LENGTH lines count)
        if(NOT count EQUAL 1)
            message(FATAL_ERROR "Invalid selected OVStage ${part} version in ${header}")
        endif()
        string(REGEX REPLACE "^#define OVSTAGE_VERSION_${part} ([0-9]+)$" "\\1" value "${lines}")
        list(APPEND parts "${value}")
    endforeach()
    list(JOIN parts "." version)
    set(${out_version} "${version}" PARENT_SCOPE)
    set(${out_header} "${header}" PARENT_SCOPE)
endfunction()

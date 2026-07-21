# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED TEST_ROOT)
    message(FATAL_ERROR "TEST_ROOT is required")
endif()
if(NOT DEFINED OVERLAY_HELPER)
    message(FATAL_ERROR "OVERLAY_HELPER is required")
endif()

include("${OVERLAY_HELPER}")

set(_plugins_dir "${TEST_ROOT}/plugins")
set(_ovstage_plugins_dir "${TEST_ROOT}/python/ovstage/bin/plugins")
set(_monolith_name "libov_25.11usd_ms.so")
set(_canonical_monolith "${_plugins_dir}/${_monolith_name}")
set(_ovstage_monolith "${_ovstage_plugins_dir}/${_monolith_name}")
set(_python_monolith "${TEST_ROOT}/python-usd/${_monolith_name}")
set(_child_script "${TEST_ROOT}/verify_overlay_child.cmake")

file(REMOVE_RECURSE "${TEST_ROOT}")
file(MAKE_DIRECTORY
    "${_plugins_dir}"
    "${_ovstage_plugins_dir}"
    "${TEST_ROOT}/python-usd")
file(WRITE "${_canonical_monolith}" "pyless-usd")
file(CREATE_LINK "${_canonical_monolith}" "${_ovstage_monolith}" COPY_ON_ERROR)
file(WRITE "${_python_monolith}" "python-usd")
file(SHA256 "${_canonical_monolith}" _original_sha256)

file(WRITE "${_child_script}" [=[
foreach(_path IN ITEMS "${CANONICAL_MONOLITH}" "${OVSTAGE_MONOLITH}")
    file(READ "${_path}" _contents)
    if(NOT _contents STREQUAL "python-usd")
        message(FATAL_ERROR "Python USD overlay was not active at ${_path}")
    endif()
endforeach()
message(FATAL_ERROR "intentional child failure")
]=])

ovphysx_run_with_python_usd_overlay(
    MONOLITH "${_python_monolith}"
    DESTINATIONS "${_plugins_dir}" "${_ovstage_plugins_dir}"
    COMMAND
        "${CMAKE_COMMAND}"
        "-DCANONICAL_MONOLITH=${_canonical_monolith}"
        "-DOVSTAGE_MONOLITH=${_ovstage_monolith}"
        -P "${_child_script}"
    RESULT_VARIABLE _child_result)

if("${_child_result}" STREQUAL "0")
    message(FATAL_ERROR "The intentional child failure was not propagated")
endif()

foreach(_path IN ITEMS "${_canonical_monolith}" "${_ovstage_monolith}")
    if(IS_SYMLINK "${_path}")
        message(FATAL_ERROR "Python USD symlink was not removed: ${_path}")
    endif()
    file(READ "${_path}" _contents)
    if(NOT _contents STREQUAL "pyless-usd")
        message(FATAL_ERROR "Py-less USD monolith was not restored at ${_path}")
    endif()
    file(SHA256 "${_path}" _restored_sha256)
    if(NOT _restored_sha256 STREQUAL _original_sha256)
        message(FATAL_ERROR "Restored USD monolith hash changed at ${_path}")
    endif()
    if(EXISTS "${_path}.ovphysx_pyless_backup")
        message(FATAL_ERROR "Overlay backup was not removed: ${_path}.ovphysx_pyless_backup")
    endif()
endforeach()

if(UNIX AND NOT APPLE)
    execute_process(
        COMMAND stat -c %d:%i "${_canonical_monolith}"
        OUTPUT_VARIABLE _canonical_identity
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE _canonical_stat_result)
    execute_process(
        COMMAND stat -c %d:%i "${_ovstage_monolith}"
        OUTPUT_VARIABLE _ovstage_identity
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE _ovstage_stat_result)
    if(NOT _canonical_stat_result EQUAL 0 OR NOT _ovstage_stat_result EQUAL 0)
        message(FATAL_ERROR "Failed to inspect restored USD monolith identities")
    endif()
    if(NOT _canonical_identity STREQUAL _ovstage_identity)
        message(FATAL_ERROR "Restored USD monolith paths no longer share one inode")
    endif()
endif()

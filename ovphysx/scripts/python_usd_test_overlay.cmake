# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

include_guard(GLOBAL)

function(_ovphysx_restore_python_usd_overlay DESTINATIONS MONOLITH_NAME)
    foreach(_destination IN LISTS DESTINATIONS)
        set(_original "${_destination}/${MONOLITH_NAME}")
        set(_backup "${_original}.ovphysx_pyless_backup")
        if(EXISTS "${_backup}" OR IS_SYMLINK "${_backup}")
            if(EXISTS "${_original}" OR IS_SYMLINK "${_original}")
                file(REMOVE "${_original}")
            endif()
            file(RENAME "${_backup}" "${_original}")
        endif()
    endforeach()
endfunction()

function(ovphysx_run_with_python_usd_overlay)
    set(_one_value_args MONOLITH RESULT_VARIABLE)
    set(_multi_value_args DESTINATIONS COMMAND)
    cmake_parse_arguments(PARSE_ARGV 0 _OVERLAY "" "${_one_value_args}" "${_multi_value_args}")

    if(NOT _OVERLAY_MONOLITH OR NOT EXISTS "${_OVERLAY_MONOLITH}")
        message(FATAL_ERROR "Python USD monolith not found: ${_OVERLAY_MONOLITH}")
    endif()
    if(NOT _OVERLAY_DESTINATIONS)
        message(FATAL_ERROR "Python USD overlay requires at least one destination")
    endif()
    if(NOT _OVERLAY_COMMAND)
        message(FATAL_ERROR "Python USD overlay requires a child command")
    endif()
    if(NOT _OVERLAY_RESULT_VARIABLE)
        message(FATAL_ERROR "Python USD overlay requires RESULT_VARIABLE")
    endif()

    get_filename_component(_monolith_name "${_OVERLAY_MONOLITH}" NAME)

    # Recover from a previously interrupted wrapper before validating the tree.
    _ovphysx_restore_python_usd_overlay("${_OVERLAY_DESTINATIONS}" "${_monolith_name}")

    foreach(_destination IN LISTS _OVERLAY_DESTINATIONS)
        set(_original "${_destination}/${_monolith_name}")
        if(NOT EXISTS "${_original}" AND NOT IS_SYMLINK "${_original}")
            message(FATAL_ERROR "Py-less USD monolith not found: ${_original}")
        endif()
    endforeach()

    set(_overlay_result 0)
    foreach(_destination IN LISTS _OVERLAY_DESTINATIONS)
        set(_original "${_destination}/${_monolith_name}")
        set(_backup "${_original}.ovphysx_pyless_backup")
        file(RENAME "${_original}" "${_backup}")

        if(WIN32)
            execute_process(
                COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${_OVERLAY_MONOLITH}" "${_original}"
                RESULT_VARIABLE _stage_result)
        else()
            execute_process(
                COMMAND "${CMAKE_COMMAND}" -E create_symlink "${_OVERLAY_MONOLITH}" "${_original}"
                RESULT_VARIABLE _stage_result)
        endif()

        if(NOT "${_stage_result}" STREQUAL "0")
            set(_overlay_result "${_stage_result}")
            break()
        endif()
    endforeach()

    if("${_overlay_result}" STREQUAL "0")
        execute_process(
            COMMAND ${_OVERLAY_COMMAND}
            RESULT_VARIABLE _overlay_result)
    endif()

    # execute_process returns normally for any child exit code, so restoration
    # always runs before the caller decides whether the test failure is fatal.
    _ovphysx_restore_python_usd_overlay("${_OVERLAY_DESTINATIONS}" "${_monolith_name}")
    set(${_OVERLAY_RESULT_VARIABLE} "${_overlay_result}" PARENT_SCOPE)
endfunction()

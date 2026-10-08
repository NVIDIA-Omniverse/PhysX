# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-BENCHMARK-001
# @covers AC-1 AC-4
#
# Testable policy and report helpers for test_benchmarks_cpp.cmake.

function(ovphysx_benchmark_driver_configure
         _RUN_GPU
         _RUN_CPU
         _RUN_CPU_ST
         _HIDDEN_VALUE
         _EXPECT_ROWS_VALUE)
    set(_ENABLED_PASSES 0)
    foreach(_RUN_PASS "${_RUN_GPU}" "${_RUN_CPU}" "${_RUN_CPU_ST}")
        if(_RUN_PASS)
            math(EXPR _ENABLED_PASSES "${_ENABLED_PASSES} + 1")
        endif()
    endforeach()

    if(_ENABLED_PASSES EQUAL 0)
        message(FATAL_ERROR "At least one benchmark pass must be enabled; empty pass selection is invalid.")
    endif()

    string(TOLOWER "${_HIDDEN_VALUE}" _HIDDEN_LOWER)
    if(_HIDDEN_LOWER STREQUAL "" OR
       _HIDDEN_LOWER STREQUAL "0" OR
       _HIDDEN_LOWER STREQUAL "false" OR
       _HIDDEN_LOWER STREQUAL "off" OR
       _HIDDEN_LOWER STREQUAL "no")
        set(_HIDDEN FALSE)
    elseif(_HIDDEN_LOWER STREQUAL "1" OR
           _HIDDEN_LOWER STREQUAL "true" OR
           _HIDDEN_LOWER STREQUAL "on" OR
           _HIDDEN_LOWER STREQUAL "yes")
        set(_HIDDEN TRUE)
    else()
        message(FATAL_ERROR
            "BENCHMARK_HIDDEN must be 1/true/on/yes or 0/false/off/no; got '${_HIDDEN_VALUE}'.")
    endif()

    set(_EXPECT_ROWS "")
    if(NOT "${_EXPECT_ROWS_VALUE}" STREQUAL "")
        if(NOT "${_EXPECT_ROWS_VALUE}" MATCHES "^[1-9][0-9]*$")
            message(FATAL_ERROR
                "BENCHMARK_EXPECT_ROWS must be a positive integer; got '${_EXPECT_ROWS_VALUE}'.")
        endif()
        if(NOT _ENABLED_PASSES EQUAL 1)
            message(FATAL_ERROR
                "BENCHMARK_EXPECT_ROWS requires exactly one benchmark pass; "
                "${_ENABLED_PASSES} passes are enabled.")
        endif()
        set(_EXPECT_ROWS "${_EXPECT_ROWS_VALUE}")
    endif()

    set(BENCHMARK_DRIVER_ENABLED_PASSES "${_ENABLED_PASSES}" PARENT_SCOPE)
    set(BENCHMARK_DRIVER_HIDDEN "${_HIDDEN}" PARENT_SCOPE)
    set(BENCHMARK_DRIVER_EXPECT_ROWS "${_EXPECT_ROWS}" PARENT_SCOPE)
endfunction()


function(ovphysx_benchmark_driver_append_hidden _ARGS_VARIABLE)
    set(_ARGS "${${_ARGS_VARIABLE}}")
    if(BENCHMARK_DRIVER_HIDDEN)
        list(APPEND _ARGS "--hidden")
    endif()
    set(${_ARGS_VARIABLE} "${_ARGS}" PARENT_SCOPE)
endfunction()


function(ovphysx_benchmark_driver_write_status _STATUS_FILE _RETURN_CODE)
    string(REPLACE "\\" "\\\\" _RETURN_CODE_JSON "${_RETURN_CODE}")
    string(REPLACE "\"" "\\\"" _RETURN_CODE_JSON "${_RETURN_CODE_JSON}")
    file(WRITE "${_STATUS_FILE}" "{\"return_code\":\"${_RETURN_CODE_JSON}\"}\n")
endfunction()


function(ovphysx_benchmark_driver_require_rows _REPORT _EXPECTED_ROWS _LABEL)
    if(NOT "${_EXPECTED_ROWS}" MATCHES "^[1-9][0-9]*$")
        message(FATAL_ERROR "Expected row count must be a positive integer; got '${_EXPECTED_ROWS}'.")
    endif()
    if(NOT EXISTS "${_REPORT}" OR IS_DIRECTORY "${_REPORT}")
        message(FATAL_ERROR "${_LABEL} report not found: ${_REPORT}")
    endif()

    file(STRINGS "${_REPORT}" _REPORT_LINES)
    set(_POSITIVE_ROWS 0)
    set(_REGENERATE_ROW
        "^[^ \t]+[ \t]+([0-9]+)[ \t]+\\(\\+/-[0-9]+\\)[ \t]+[0-9]+[ \t]*$")
    set(_COMPARE_ROW
        "^[^ \t]+[ \t]+([0-9]+)[ \t]+\\([ \t]*[+-][0-9]+\\.[0-9]+%\\)[ \t]+[0-9]+[ \t]+\\([ \t]*[+-][0-9]+\\.[0-9]+%\\)([ \t]+(No baseline|FAIL|Skipped))?[ \t]*$")

    foreach(_LINE IN LISTS _REPORT_LINES)
        set(_AVERAGE_US 0)
        set(_IS_POSITIVE_ROW FALSE)
        if(_LINE MATCHES "${_REGENERATE_ROW}")
            set(_AVERAGE_US "${CMAKE_MATCH_1}")
            set(_IS_POSITIVE_ROW TRUE)
        elseif(_LINE MATCHES "${_COMPARE_ROW}")
            set(_AVERAGE_US "${CMAKE_MATCH_1}")
            if(NOT _LINE MATCHES "[ \t]+(FAIL|Skipped)[ \t]*$")
                set(_IS_POSITIVE_ROW TRUE)
            endif()
        endif()

        if(_IS_POSITIVE_ROW AND _AVERAGE_US GREATER 0)
            math(EXPR _POSITIVE_ROWS "${_POSITIVE_ROWS} + 1")
        endif()
    endforeach()

    if(NOT _POSITIVE_ROWS EQUAL _EXPECTED_ROWS)
        message(FATAL_ERROR
            "${_LABEL} report expected exactly ${_EXPECTED_ROWS} positive data row(s), "
            "found ${_POSITIVE_ROWS}: ${_REPORT}")
    endif()
    message(STATUS
        "${_LABEL} report contains exactly ${_POSITIVE_ROWS} positive data row(s): ${_REPORT}")
endfunction()

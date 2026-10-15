# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PACKAGING-TESTRESULT-001
# @covers AC-1 AC-2 AC-3

cmake_minimum_required(VERSION 3.16)
get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
if(NOT DEFINED TEST_ROOT)
    set(TEST_ROOT "${PROJECT_ROOT}/_build/tests/result_guards")
endif()
file(MAKE_DIRECTORY "${TEST_ROOT}")

# Execute the actual parser bodies without bootstrapping an SDK, CUDA or uv.
# Fail on structural drift instead of testing a copied implementation.
foreach(driver cpp python_runtime)
    file(READ "${PROJECT_ROOT}/scripts/test_${driver}.cmake" source)
    if(driver STREQUAL "cpp")
        set(parser parse_gtest_results)
        set(allowlist GTEST_PASSES_KNOWN_FULLY_SKIPPED)
    else()
        set(parser parse_pytest_junit)
        set(allowlist PYTHON_SUITES_KNOWN_FULLY_SKIPPED)
    endif()
    string(FIND "${source}" "function(${parser} " begin)
    if(begin EQUAL -1)
        message(FATAL_ERROR "Cannot find ${parser}")
    endif()
    string(SUBSTRING "${source}" ${begin} -1 tail)
    string(FIND "${tail}" "endfunction()" end)
    if(end EQUAL -1)
        message(FATAL_ERROR "Cannot find end of ${parser}")
    endif()
    math(EXPR length "${end} + 13")
    string(SUBSTRING "${tail}" 0 ${length} body)
    string(REGEX MATCH "set\\(${allowlist}[^\n]*\\)" setting "${source}")
    if(NOT setting)
        message(FATAL_ERROR "Cannot find ${allowlist}")
    endif()
    set(${driver}_parser "${parser}")
    set(${driver}_body "${setting}\n${body}")
endforeach()

function(check_result name driver label attributes expected message_pattern)
    set(xml "${TEST_ROOT}/${name}.xml")
    set(script "${TEST_ROOT}/${name}.cmake")
    file(WRITE "${xml}" "<testsuites ${attributes}>${ARGN}</testsuites>")
    file(WRITE "${script}" "cmake_minimum_required(VERSION 3.16)\n${${driver}_body}\n${${driver}_parser}(\"${label}\" \"${xml}\")\n")
    execute_process(COMMAND "${CMAKE_COMMAND}" -P "${script}"
        RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE error)
    if(expected STREQUAL "pass")
        if(NOT result STREQUAL "0")
            message(FATAL_ERROR "${name}: unexpected failure: ${output}${error}")
        endif()
    elseif(result STREQUAL "0")
        message(FATAL_ERROR "${name}: unexpected success: ${output}${error}")
    endif()
    string(REGEX REPLACE "[ \t\r\n]+" " " diagnostic "${output}${error}")
    if(NOT "${diagnostic}" MATCHES "${message_pattern}")
        message(FATAL_ERROR "${name}: missing diagnostic '${message_pattern}': ${output}${error}")
    endif()
    message(STATUS "PASS: ${name}")
endfunction()

foreach(driver cpp python_runtime)
    check_result(${driver}_all_skipped ${driver} ordinary "tests=\"2\" skipped=\"2\"" fail "no cases completed successfully")
    check_result(${driver}_healthy ${driver} ordinary "tests=\"3\"" pass "3 passed")
    check_result(${driver}_partial_skip ${driver} ordinary "tests=\"3\" skipped=\"1\"" pass "2 passed")
    check_result(${driver}_zero ${driver} ordinary "tests=\"0\"" fail "zero tests")
    check_result(${driver}_failure ${driver} ordinary "tests=\"2\" failures=\"1\"" fail "had failures")
    check_result(${driver}_malformed ${driver} ordinary "skipped=\"1\"" fail "malformed")
    check_result(${driver}_inconsistent ${driver} ordinary "tests=\"1\" skipped=\"2\"" fail "inconsistent counts")
endforeach()
check_result(cpp_known_skip cpp cpu-no-cuda-context "tests=\"1\" skipped=\"1\"" pass "known fully-skipped exception")
check_result(cpp_known_failure cpp cpu-no-cuda-context "tests=\"1\" failures=\"1\"" fail "had failures")
check_result(cpp_known_zero cpp cpu-no-cuda-context "tests=\"0\"" fail "zero tests")
check_result(cpp_disabled cpp ordinary "tests=\"1\" disabled=\"1\"" fail "no cases completed successfully")
check_result(python_error python_runtime ordinary "tests=\"1\" errors=\"1\"" fail "had failures or errors")
check_result(python_no_exception python_runtime cpu-no-cuda-context "tests=\"1\" skipped=\"1\"" fail "no cases completed successfully")
# Match the real GTest XML shape, including the location-prefixed skip message.
set(hardware_reason "needs at least two CUDA devices to select a non-zero ordinal")
set(hardware_case "<testcase name=\"DirectGpuHostReadOnNonZeroOrdinal\" classname=\"ActiveCudaGpusAttachTest\"><skipped message=\"test_instantiation.cpp:374&#x0A;${hardware_reason}&#x0A;\"/></testcase>")
check_result(cpp_hardware_skip cpp cuda-selection-nonzero-ordinal "tests=\"1\" skipped=\"1\"" pass "hardware prerequisite.*scenario was not exercised" "${hardware_case}")
check_result(cpp_hardware_wrong_label cpp ordinary "tests=\"1\" skipped=\"1\"" fail "no cases completed successfully" "${hardware_case}")
check_result(cpp_hardware_missing_reason cpp cuda-selection-nonzero-ordinal "tests=\"1\" skipped=\"1\"" fail "no cases completed successfully")
string(REPLACE "${hardware_reason}" "unexpected runtime skip" wrong_reason "${hardware_case}")
check_result(cpp_hardware_wrong_reason cpp cuda-selection-nonzero-ordinal "tests=\"1\" skipped=\"1\"" fail "no cases completed successfully" "${wrong_reason}")
string(REPLACE "DirectGpuHostReadOnNonZeroOrdinal" "OtherTest" wrong_case "${hardware_case}")
check_result(cpp_hardware_wrong_case cpp cuda-selection-nonzero-ordinal "tests=\"1\" skipped=\"1\"" fail "no cases completed successfully" "${wrong_case}")
check_result(cpp_hardware_disabled cpp cuda-selection-nonzero-ordinal "tests=\"1\" disabled=\"1\"" fail "no cases completed successfully" "${hardware_case}")
check_result(cpp_hardware_failure cpp cuda-selection-nonzero-ordinal "tests=\"1\" failures=\"1\"" fail "had failures" "${hardware_case}")
check_result(cpp_hardware_zero cpp cuda-selection-nonzero-ordinal "tests=\"0\"" fail "zero tests" "${hardware_case}")
check_result(cpp_hardware_extra_case cpp cuda-selection-nonzero-ordinal "tests=\"2\" skipped=\"2\"" fail "no cases completed successfully" "${hardware_case}${hardware_case}")
check_result(cpp_hardware_healthy cpp cuda-selection-nonzero-ordinal "tests=\"1\"" pass "1 passed")
# The isolated solver cases deliberately skip beyond the host's tasking capacity.
# GoogleTest escapes the apostrophe and reports the skip on the child suite.
set(solver_reason "Requested count exceeds this host&apos;s default tasking capacity")
foreach(workers 1 2 4 8)
    set(solver_case "<testcase name=\"Workers${workers}\" classname=\"SolverThreadCount\"><skipped message=\"test_solver_thread_count.cpp:38&#x0A;${solver_reason}&#x0A;\"/></testcase>")
    check_result(cpp_solver${workers}_skip cpp solver-threads-${workers} "tests=\"1\" failures=\"0\" disabled=\"0\"" pass "hardware prerequisite.*scenario was not exercised"
        "<testsuite name=\"SolverThreadCount\" tests=\"1\" skipped=\"1\">${solver_case}</testsuite>")
endforeach()
check_result(cpp_solver_healthy cpp solver-threads-8 "tests=\"1\"" pass "1 passed")
check_result(cpp_solver_wrong_label cpp ordinary "tests=\"1\" skipped=\"1\"" fail "no cases completed successfully" "${solver_case}")
check_result(cpp_solver_label_suffix cpp solver-threads-8-extra "tests=\"1\" skipped=\"1\"" fail "no cases completed successfully" "${solver_case}")
string(REPLACE "Workers8" "Workers3" unsupported_count "${solver_case}")
check_result(cpp_solver_unsupported_count cpp solver-threads-3 "tests=\"1\" skipped=\"1\"" fail "no cases completed successfully" "${unsupported_count}")
check_result(cpp_solver_wrong_count cpp solver-threads-4 "tests=\"1\" skipped=\"1\"" fail "no cases completed successfully" "${solver_case}")
string(REPLACE "Workers8" "OtherTest" wrong_case "${solver_case}")
check_result(cpp_solver_wrong_case cpp solver-threads-8 "tests=\"1\" skipped=\"1\"" fail "no cases completed successfully" "${wrong_case}")
string(REPLACE "SolverThreadCount" "OtherSuite" wrong_suite "${solver_case}")
check_result(cpp_solver_wrong_suite cpp solver-threads-8 "tests=\"1\" skipped=\"1\"" fail "no cases completed successfully" "${wrong_suite}")
check_result(cpp_solver_missing_reason cpp solver-threads-8 "tests=\"1\" skipped=\"1\"" fail "no cases completed successfully")
string(REPLACE "${solver_reason}" "unexpected runtime skip" wrong_reason "${solver_case}")
check_result(cpp_solver_wrong_reason cpp solver-threads-8 "tests=\"1\" skipped=\"1\"" fail "no cases completed successfully" "${wrong_reason}")
string(REPLACE "${solver_reason}" "${solver_reason} plus an unrelated reason" extended_reason "${solver_case}")
check_result(cpp_solver_extended_reason cpp solver-threads-8 "tests=\"1\" skipped=\"1\"" fail "no cases completed successfully" "${extended_reason}")
check_result(cpp_solver_disabled cpp solver-threads-8 "tests=\"1\" disabled=\"1\"" fail "no cases completed successfully" "${solver_case}")
check_result(cpp_solver_failure cpp solver-threads-8 "tests=\"2\" skipped=\"1\" failures=\"1\"" fail "had failures" "${solver_case}")
check_result(cpp_solver_zero cpp solver-threads-8 "tests=\"0\"" fail "zero tests" "${solver_case}")
check_result(cpp_solver_inconsistent cpp solver-threads-8 "tests=\"1\" skipped=\"2\"" fail "inconsistent counts" "${solver_case}")
check_result(cpp_solver_extra_count cpp solver-threads-8 "tests=\"2\" skipped=\"2\"" fail "no cases completed successfully" "${solver_case}")
check_result(cpp_solver_extra_case cpp solver-threads-8 "tests=\"1\" skipped=\"1\"" fail "no cases completed successfully" "${solver_case}${solver_case}")
# GoogleTest emits skipped counts on each child testsuite, not on testsuites.
# Reading just the first child makes an entirely skipped multi-suite pass look
# partially healthy. These fixtures preserve that real report hierarchy.
set(skipped_suite "<testsuite name=\"First\" tests=\"1\" failures=\"0\" disabled=\"0\" skipped=\"1\"><testcase name=\"Skipped\" classname=\"First\"><skipped message=\"fixture prerequisite\"/></testcase></testsuite>")
set(second_skipped_suite "<testsuite skipped=\"1\" name=\"Second\" tests=\"1\" failures=\"0\" disabled=\"0\"><testcase name=\"Skipped\" classname=\"Second\"><skipped message=\"fixture prerequisite\"/></testcase></testsuite>")
check_result(cpp_nested_all_skipped cpp ordinary "tests=\"2\" failures=\"0\" disabled=\"0\"" fail "no cases completed successfully" "${skipped_suite}${second_skipped_suite}")
set(partially_skipped_suite "<testsuite name=\"First\" tests=\"2\" failures=\"0\" disabled=\"0\" skipped=\"1\"><testcase name=\"Passed\" classname=\"First\"/><testcase name=\"Skipped\" classname=\"First\"><skipped message=\"fixture prerequisite\"/></testcase></testsuite>")
check_result(cpp_nested_partial_skip cpp ordinary "tests=\"3\" failures=\"0\" disabled=\"0\"" pass "1 passed, 2 skipped" "${partially_skipped_suite}${second_skipped_suite}")
message(STATUS "All 52 result-guard cases passed")

# Exercise discovery from the real driver, replacing only the process boundaries.
file(READ "${PROJECT_ROOT}/scripts/test_cpp.cmake" source)
string(FIND "${source}" "function(run_isolated_gtest_cases " begin)
if(begin EQUAL -1)
    message(FATAL_ERROR "Cannot find run_isolated_gtest_cases")
endif()
string(SUBSTRING "${source}" ${begin} -1 tail)
string(FIND "${tail}" "endfunction()" end)
if(end EQUAL -1)
    message(FATAL_ERROR "Cannot find end of run_isolated_gtest_cases")
endif()
math(EXPR length "${end} + 13")
string(SUBSTRING "${tail}" 0 ${length} discovery_body)

set(discovery_boundaries [=[
function(execute_process)
    cmake_parse_arguments(process "" "WORKING_DIRECTORY;TIMEOUT;RESULT_VARIABLE;OUTPUT_VARIABLE;ERROR_VARIABLE" "COMMAND" ${ARGN})
    if(process_UNPARSED_ARGUMENTS OR
       NOT "${process_COMMAND}" STREQUAL "${GTEST_EXECUTABLE};--gtest_list_tests;--gtest_filter=${expected_suite}.*" OR
       NOT "${process_WORKING_DIRECTORY}" STREQUAL "${PROJECT_ROOT}" OR
       NOT "${process_TIMEOUT}" STREQUAL "60")
        message(FATAL_ERROR "Unexpected discovery process arguments: ${ARGN}")
    endif()
    set_property(GLOBAL APPEND PROPERTY discovery_suites "${expected_suite}")
    set(${process_RESULT_VARIABLE} "${fixture_result}" PARENT_SCOPE)
    set(${process_OUTPUT_VARIABLE} "${fixture_output}" PARENT_SCOPE)
    set(${process_ERROR_VARIABLE} "controlled discovery error" PARENT_SCOPE)
endfunction()

function(run_gtest_pass label filter)
    if(NOT ARGC EQUAL 2)
        message(FATAL_ERROR "Unexpected isolated pass arguments: ${ARGV}")
    endif()
    set_property(GLOBAL APPEND PROPERTY OVPHYSX_GTEST_PASS_LABELS "${label}")
    set_property(GLOBAL APPEND PROPERTY discovery_filters "${filter}")
endfunction()
]=])

function(check_discovery name fixture expected message_pattern calls labels filters suites)
    set(script "${TEST_ROOT}/discovery_${name}.cmake")
    file(WRITE "${script}"
        "cmake_minimum_required(VERSION 3.16)\n"
        "set(PROJECT_ROOT [==[${TEST_ROOT}/discovery working directory]==])\n"
        "set(GTEST_EXECUTABLE [==[${TEST_ROOT}/gtest executable]==])\n"
        "${discovery_body}\n${discovery_boundaries}\n${fixture}\n${calls}\n")
    file(APPEND "${script}"
        "get_property(labels GLOBAL PROPERTY OVPHYSX_GTEST_PASS_LABELS)\n"
        "get_property(filters GLOBAL PROPERTY discovery_filters)\n"
        "get_property(suites GLOBAL PROPERTY discovery_suites)\n"
        "if(NOT \"\${labels}\" STREQUAL \"${labels}\" OR NOT \"\${filters}\" STREQUAL \"${filters}\" OR NOT \"\${suites}\" STREQUAL \"${suites}\")\n"
        "  message(FATAL_ERROR \"Unexpected isolated passes: \${labels} / \${filters} / \${suites}\")\n"
        "endif()\nmessage(STATUS \"Discovery passes matched\")\n")
    execute_process(COMMAND "${CMAKE_COMMAND}" -P "${script}"
        RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE error)
    if(expected STREQUAL "pass")
        if(NOT result STREQUAL "0")
            message(FATAL_ERROR "discovery_${name}: unexpected failure: ${output}${error}")
        endif()
    elseif(result STREQUAL "0")
        message(FATAL_ERROR "discovery_${name}: unexpected success")
    endif()
    string(REGEX REPLACE "[ \t\r\n]+" " " diagnostic "${output}${error}")
    if(NOT "${diagnostic}" MATCHES "${message_pattern}")
        message(FATAL_ERROR "discovery_${name}: missing diagnostic '${message_pattern}': ${output}${error}")
    endif()
    message(STATUS "PASS: discovery_${name}")
endfunction()

check_discovery(lf_sequential
    "set(fixture_result 0)\nset(fixture_output \"OmniPvdGlobalConfig.\\n  First\\n  DISABLED_Ignored\\n  Last\\n\")"
    pass "Discovery passes matched"
    "set(expected_suite OmniPvdGlobalConfig)\nrun_isolated_gtest_cases(OmniPvdGlobalConfig lifecycle-omnipvd)\nset(expected_suite OmniPvdRecording)\nset(fixture_output \"OmniPvdRecording.\\n  First\\n  Tcp\\n\")\nrun_isolated_gtest_cases(OmniPvdRecording omnipvd-recording)"
    "lifecycle-omnipvd-First;lifecycle-omnipvd-Last;omnipvd-recording-First;omnipvd-recording-Tcp"
    "OmniPvdGlobalConfig.First;OmniPvdGlobalConfig.Last;OmniPvdRecording.First;OmniPvdRecording.Tcp"
    "OmniPvdGlobalConfig;OmniPvdRecording")
check_discovery(crlf
    "set(fixture_result 0)\nset(fixture_output \"OmniPvdRecording.\\r\\n  File\\r\\n  DISABLED_Ignored\\r\\n  Tcp\\r\\n\")"
    pass "Discovery passes matched"
    "set(expected_suite OmniPvdRecording)\nrun_isolated_gtest_cases(OmniPvdRecording omnipvd-recording)"
    "omnipvd-recording-File;omnipvd-recording-Tcp"
    "OmniPvdRecording.File;OmniPvdRecording.Tcp" "OmniPvdRecording")
check_discovery(empty
    "set(fixture_result 0)\nset(fixture_output \"\")"
    fail "No.*tests discovered"
    "set(expected_suite OmniPvdGlobalConfig)\nrun_isolated_gtest_cases(OmniPvdGlobalConfig lifecycle-omnipvd)"
    "" "" "OmniPvdGlobalConfig")
check_discovery(nonzero
    "set(fixture_result 7)\nset(fixture_output \"OmniPvdRecording.\\n  File\\n\")"
    fail "Cannot discover.*7.*controlled discovery error"
    "set(expected_suite OmniPvdRecording)\nrun_isolated_gtest_cases(OmniPvdRecording omnipvd-recording)"
    "" "" "OmniPvdRecording")
check_discovery(timeout
    "set(fixture_result \"Process terminated due to timeout\")\nset(fixture_output \"OmniPvdRecording.\\n  File\\n\")"
    fail "Cannot discover.*Process terminated due to timeout.*controlled discovery error"
    "set(expected_suite OmniPvdRecording)\nrun_isolated_gtest_cases(OmniPvdRecording omnipvd-recording)"
    "" "" "OmniPvdRecording")
message(STATUS "All 5 isolated-discovery cases passed")

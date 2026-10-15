# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-BENCHMARK-006
# @covers AC-1 AC-3 AC-4 AC-5
#
# Run directly with cmake -P scripts/test_output_write_contract.cmake after a
# benchmark-enabled build and SDK install. Also included by the CI contract.
cmake_minimum_required(VERSION 3.17)

function(run_output_write_contract)
    set(SCRIPT_DIR "${CMAKE_CURRENT_FUNCTION_LIST_DIR}")
    include("${SCRIPT_DIR}/crossplatform_helpers.cmake")
    include("${SCRIPT_DIR}/build_common.cmake")
    include("${SCRIPT_DIR}/fetch_test_schemas.cmake")
    include("${SCRIPT_DIR}/benchmark_driver_common.cmake")
    include("${SCRIPT_DIR}/benchmark_baseline_txn.cmake")
    set(BINARY "${STANDARD_OUTPUT_DIR}/ovphysx_benchmarks${EXE_SUFFIX}")
    set(RESULTS "${PROJECT_ROOT}/_build/output_write_contract_results")
    file(MAKE_DIRECTORY "${RESULTS}")
    set(BASELINE_TXN_TARGET "${STANDARD_OUTPUT_DIR}/../data/benchmarkData/_baseline.txt")
    set(BASELINE_TXN_BACKUP "${RESULTS}/_baseline.bak")
    set(BASELINE_TXN_MARKER "${RESULTS}/_baseline.txn")
    baseline_txn_recover()

    if(OS_NAME STREQUAL "windows")
        ovphysx_resolve_ovstage_paths()
        set(INSTALL_DIR "${PROJECT_ROOT}/_install")
        set(ENV{PATH} "${INSTALL_DIR}/bin${PATH_SEP}${INSTALL_DIR}/plugins${PATH_SEP}${INSTALL_DIR}/plugins/bin/deps${PATH_SEP}${PROJECT_ROOT}/_build/target-deps/python${PATH_SEP}${OVPHYSX_OVSTAGE_RUNTIME_DIR}${PATH_SEP}${OVPHYSX_OVSTAGE_RUNTIME_DIR}/plugins${PATH_SEP}$ENV{PATH}")
        set(ENV{OVPHYSX_LIB} "${INSTALL_DIR}/bin/ovphysx.dll")
    endif()

    set(ROWS
        OutputWrite.tensorbinding_rb_8192_cpu
        OutputWrite.tensorbinding_property_1024_cpu
        OutputWrite.tensorbinding_arti_root_8192_cpu
        OutputWrite.tensorbinding_arti_dof_8192_cpu
        OutputWrite.writeonly_shape_rb_1024_cpu
        OutputWrite.writeonly_rb_instancer_1024_cpu
        OutputWrite.writeonly_tendon_fixed_1024_cpu
        OutputWrite.writeonly_tendon_spatial_8192_cpu
    )
    list(JOIN ROWS ":" FILTER)
    set(REPORT "${RESULTS}/positive.txt")
    file(REMOVE "${REPORT}")
    baseline_txn_begin()
    execute_process(
        COMMAND "${BINARY}" "--regenerate" "--filter=${FILTER}"
            "--steps=2" "--runs=3" "--data=${PROJECT_ROOT}/tests/data"
            "--report=${REPORT}"
        WORKING_DIRECTORY "${PROJECT_ROOT}"
        TIMEOUT 1800
        RESULT_VARIABLE RC
        OUTPUT_VARIABLE STDOUT
        ERROR_VARIABLE STDERR
    )
    baseline_txn_commit()
    file(WRITE "${RESULTS}/positive.log" "${STDOUT}${STDERR}")
    if(NOT "${RC}" STREQUAL "0")
        message(FATAL_ERROR "OutputWrite contract failed (${RC}); see ${RESULTS}/positive.log")
    endif()
    if("${STDOUT}${STDERR}" MATCHES "[Cc]ontact buffer overflow|[Pp]atch buffer overflow")
        message(FATAL_ERROR "OutputWrite contract overflowed contact capacity; see ${RESULTS}/positive.log")
    endif()
    list(LENGTH ROWS EXPECTED_ROWS)
    ovphysx_benchmark_driver_require_rows("${REPORT}" "${EXPECTED_ROWS}" "OutputWrite")

    set(ORDINARY "${RESULTS}/ordinary")
    file(MAKE_DIRECTORY "${ORDINARY}/data" "${ORDINARY}/benchmarks/data")
    file(WRITE "${ORDINARY}/benchmarks/data/cubes20_instancer_envs_1024.usda"
        "#usda 1.0\n(\n    subLayers = [@${PROJECT_ROOT}/tests/benchmarks/data/cubes20_write_cpu.usda@]\n)\n")
    function(run_output_write_negative CASE NEGATIVE_ROW DATA EXPECTED_ERROR)
        set(REPORT "${RESULTS}/${CASE}.txt")
        file(REMOVE "${REPORT}")
        baseline_txn_begin()
        execute_process(
            COMMAND "${BINARY}" "--regenerate" "--filter=${NEGATIVE_ROW}"
                "--steps=2" "--runs=3" "--data=${DATA}" "--report=${REPORT}"
            WORKING_DIRECTORY "${PROJECT_ROOT}"
            TIMEOUT 300
            RESULT_VARIABLE RC
            OUTPUT_VARIABLE STDOUT
            ERROR_VARIABLE STDERR
        )
        baseline_txn_commit()
        file(WRITE "${RESULTS}/${CASE}.log" "${STDOUT}${STDERR}")
        if("${RC}" STREQUAL "0" OR NOT "${STDOUT}${STDERR}" MATCHES "FAILURE.*${NEGATIVE_ROW}"
            OR NOT "${STDOUT}${STDERR}" MATCHES "${EXPECTED_ERROR}")
            message(FATAL_ERROR "OutputWrite ${CASE} contract did not reject its invalid scene; see ${RESULTS}/${CASE}.log")
        endif()
        if(EXISTS "${REPORT}")
            file(READ "${REPORT}" TEXT)
            string(FIND "${TEXT}" "${NEGATIVE_ROW} " FOUND)
            if(NOT FOUND EQUAL -1)
                message(FATAL_ERROR "OutputWrite ${CASE} contract published a failed row")
            endif()
        endif()
    endfunction()
    run_output_write_negative(missing_data OutputWrite.writeonly_tendon_fixed_1024_cpu
        "${RESULTS}/missing/data" "ovstage load failed")
    run_output_write_negative(ordinary_instancer OutputWrite.writeonly_rb_instancer_1024_cpu
        "${ORDINARY}/data" "expected one instancer with twenty instances")
    message(STATUS "OutputWrite contract: PASSED")
endfunction()

run_output_write_contract()

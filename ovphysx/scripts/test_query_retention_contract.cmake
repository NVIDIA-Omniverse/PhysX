# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-BENCHMARK-008
# @covers AC-1 AC-5
#
# @implements REQ-CAPI-BENCHMARK-009
# @covers AC-1 AC-4 AC-5

# Included by test_benchmark_contract.cmake after its common setup. Keep the
# original inventory selections and their positive runs unchanged.
if(NOT COMMAND run_verify_inventory OR NOT COMMAND baseline_txn_begin)
    message(FATAL_ERROR "Run scripts/test_benchmark_contract.cmake for the caller diagnostics contract")
endif()
set(QUERY_RETENTION_INVENTORY "${INVENTORY_TESTS_DIR}/query_retention_inventory.json")
set(BATCHED_WRITE_INVENTORY "${INVENTORY_TESTS_DIR}/batched_write_inventory.json")
run_verify_inventory(check-union
    "${PRODUCER_INVENTORY_JSON}" "${OUTPUT_READ_INVENTORY_JSON}"
    "${CONTACT_INVENTORY_JSON}" "${HIGH_SCALE_INVENTORY_JSON}"
    "${WRITE_ONLY_INVENTORY_JSON}" "${QUERY_RETENTION_INVENTORY}" "${BATCHED_WRITE_INVENTORY}")

foreach(_QR_DIAGNOSTIC query_retention batched_write)
    if(_QR_DIAGNOSTIC STREQUAL "query_retention")
        set(_QR_FAMILY "QueryRetention")
        set(_QR_INVENTORY "${QUERY_RETENTION_INVENTORY}")
    else()
        set(_QR_FAMILY "BatchedWrite")
        set(_QR_INVENTORY "${BATCHED_WRITE_INVENTORY}")
    endif()
    run_verify_inventory(filter "${_QR_INVENTORY}" --device cpu)
    set(_QR_FILTER "${VERIFY_STDOUT}")
    if(NOT _QR_FILTER)
        message(FATAL_ERROR "${_QR_FAMILY} inventory produced no CPU rows")
    endif()
    set(_QR_LIST "${RESULTS_DIR}/${_QR_DIAGNOSTIC}_list.txt")
    execute_process(
        COMMAND "${BENCHMARK_BINARY}" --list "--filter=${_QR_FAMILY}.*"
        WORKING_DIRECTORY "${PROJECT_ROOT}" TIMEOUT 120
        RESULT_VARIABLE _QR_RC OUTPUT_FILE "${_QR_LIST}" ERROR_VARIABLE _QR_STDERR)
    if(NOT _QR_RC EQUAL 0)
        message(FATAL_ERROR "${_QR_FAMILY} listing failed (${_QR_RC}): ${_QR_STDERR}")
    endif()
    run_verify_inventory(check-list "${_QR_INVENTORY}" "${_QR_LIST}")

    foreach(_QR_MODE vacuity gpu positive)
        set(_QR_REPORT "${RESULTS_DIR}/${_QR_DIAGNOSTIC}_${_QR_MODE}_report.txt")
        set(_QR_LOG "${RESULTS_DIR}/${_QR_DIAGNOSTIC}_${_QR_MODE}.log")
        file(REMOVE "${_QR_REPORT}" "${_QR_LOG}")
        set(_QR_ARGS "--filter=${_QR_FAMILY}.*" "--data=${PROJECT_ROOT}/tests/data"
            "--report=${_QR_REPORT}" --verbose)
        if(_QR_MODE STREQUAL "gpu")
            list(APPEND _QR_ARGS --hidden --forceGpu)
        elseif(_QR_MODE STREQUAL "positive")
            list(APPEND _QR_ARGS --hidden --regenerate --steps=1 --runs=3)
            baseline_txn_begin()
        endif()
        execute_process(
            COMMAND "${BENCHMARK_BINARY}" ${_QR_ARGS}
            WORKING_DIRECTORY "${PROJECT_ROOT}" TIMEOUT 900
            RESULT_VARIABLE _QR_RC OUTPUT_VARIABLE _QR_STDOUT ERROR_VARIABLE _QR_STDERR)
        if(_QR_MODE STREQUAL "positive")
            baseline_txn_commit()
        endif()
        file(WRITE "${_QR_LOG}" "${_QR_STDOUT}${_QR_STDERR}")
        if(NOT _QR_RC EQUAL 0)
            message(FATAL_ERROR "${_QR_FAMILY} ${_QR_MODE} failed (${_QR_RC}); log: ${_QR_LOG}")
        endif()
        if(_QR_MODE STREQUAL "positive")
            run_verify_inventory(check-report "${_QR_INVENTORY}" "${_QR_REPORT}" --device cpu)
        elseif(_QR_MODE STREQUAL "gpu")
            string(REPLACE ":" ";" _QR_GPU_ROWS "${_QR_FILTER}")
            list(TRANSFORM _QR_GPU_ROWS APPEND "_GPU")
            run_verify_inventory(check-skipped "${_QR_LOG}" ${_QR_GPU_ROWS})
            run_verify_inventory(check-unpublished "${_QR_REPORT}" ${_QR_GPU_ROWS})
        else()
            run_verify_inventory(check-vacuity "${_QR_INVENTORY}" "${_QR_REPORT}")
        endif()
    endforeach()
    message(STATUS "ovphysx ${_QR_FAMILY} diagnostics contract: PASSED")
endforeach()

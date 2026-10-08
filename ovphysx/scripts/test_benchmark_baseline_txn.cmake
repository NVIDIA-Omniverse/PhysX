# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-BENCHMARK-001
# @covers AC-4
#
# Focused regression for scripts/benchmark_baseline_txn.cmake.
#
# Everything runs against a scratch directory under _build/, never the real
# developer baseline, so the recovery path can be exercised by hand-seeding the
# exact on-disk state a killed contract run would leave behind.
#
#   cmake -P scripts/test_benchmark_baseline_txn.cmake
#
# Cases that are supposed to abort are re-entered as child cmake processes
# (-DTXN_CASE=...), because message(FATAL_ERROR) cannot be caught in-process.

cmake_minimum_required(VERSION 3.16)

get_filename_component(SCRIPT_DIR "${CMAKE_CURRENT_LIST_DIR}" ABSOLUTE)
get_filename_component(PROJECT_ROOT "${SCRIPT_DIR}/.." ABSOLUTE)
include("${SCRIPT_DIR}/benchmark_baseline_txn.cmake")

set(SCRATCH_DIR "${PROJECT_ROOT}/_build/benchmark_contract_results/baseline_txn_test")
set(BASELINE_TXN_TARGET "${SCRATCH_DIR}/_baseline.txt")
set(BASELINE_TXN_BACKUP "${SCRATCH_DIR}/_baseline.developer.bak")
set(BASELINE_TXN_MARKER "${SCRATCH_DIR}/_baseline.txn")

set(DEVELOPER_CONTENT "Authoring.population_add_drip_cpu 1.234\nWriteScaling.stage_1k_cpu 5.678\n")
set(NARROW_CONTENT "Authoring.teleport_tensor_cpu 9.999\n")


function(reset_scratch)
    file(REMOVE_RECURSE "${SCRATCH_DIR}")
    file(MAKE_DIRECTORY "${SCRATCH_DIR}")
endfunction()


function(expect_missing what path)
    if(EXISTS "${path}")
        message(FATAL_ERROR "FAIL ${what}: '${path}' should not exist")
    endif()
endfunction()


function(expect_content what path expected)
    if(NOT EXISTS "${path}")
        message(FATAL_ERROR "FAIL ${what}: '${path}' is missing")
    endif()
    file(READ "${path}" _actual)
    if(NOT _actual STREQUAL "${expected}")
        message(FATAL_ERROR "FAIL ${what}: '${path}' holds\n---\n${_actual}\n---\nexpected\n---\n${expected}\n---")
    endif()
endfunction()


function(expect_no_open_transaction what)
    expect_missing("${what} (marker)" "${BASELINE_TXN_MARKER}")
    expect_missing("${what} (backup)" "${BASELINE_TXN_BACKUP}")
endfunction()


# Reproduces the state a SIGKILL leaves behind: the transaction was opened, the
# benchmark child already rewrote the target, and nothing restored it.
function(seed_interrupted_transaction had_baseline)
    reset_scratch()
    file(WRITE "${BASELINE_TXN_TARGET}" "${NARROW_CONTENT}")
    if(had_baseline)
        file(WRITE "${BASELINE_TXN_BACKUP}" "${DEVELOPER_CONTENT}")
        file(SHA256 "${BASELINE_TXN_BACKUP}" _hash)
        file(WRITE "${BASELINE_TXN_MARKER}" "EXISTED ${_hash}\n")
    else()
        file(WRITE "${BASELINE_TXN_MARKER}" "ABSENT\n")
    endif()
endfunction()


function(run_child_case case_name expect_success)
    execute_process(
        COMMAND "${CMAKE_COMMAND}" "-DTXN_CASE=${case_name}" -P "${CMAKE_CURRENT_LIST_FILE}"
        OUTPUT_VARIABLE _out
        ERROR_VARIABLE _err
        RESULT_VARIABLE _rc
    )
    if(expect_success AND NOT _rc EQUAL 0)
        message(FATAL_ERROR "FAIL case '${case_name}' should have succeeded (rc=${_rc}):\n${_out}\n${_err}")
    endif()
    if(NOT expect_success AND _rc EQUAL 0)
        message(FATAL_ERROR "FAIL case '${case_name}' should have aborted but exited 0:\n${_out}\n${_err}")
    endif()
    set(CHILD_STDERR "${_err}" PARENT_SCOPE)
endfunction()


# ---------------------------------------------------------------------------
# Child cases: each is expected to abort.
# ---------------------------------------------------------------------------
if(TXN_CASE STREQUAL "begin_over_unrecovered_backup")
    seed_interrupted_transaction(TRUE)
    baseline_txn_begin()
    message(FATAL_ERROR "unreachable: begin() accepted an unrecovered transaction")
elseif(TXN_CASE STREQUAL "commit_without_begin")
    reset_scratch()
    file(WRITE "${BASELINE_TXN_TARGET}" "${DEVELOPER_CONTENT}")
    baseline_txn_commit()
    message(FATAL_ERROR "unreachable: commit() accepted a closed transaction")
elseif(TXN_CASE STREQUAL "commit_detects_corrupted_snapshot")
    reset_scratch()
    file(WRITE "${BASELINE_TXN_TARGET}" "${DEVELOPER_CONTENT}")
    baseline_txn_begin()
    file(WRITE "${BASELINE_TXN_TARGET}" "${NARROW_CONTENT}")
    # Corrupting the snapshot is the only way to make the restore disagree with
    # the recorded hash. The identity check has to notice.
    file(WRITE "${BASELINE_TXN_BACKUP}" "corrupted\n")
    baseline_txn_commit()
    message(FATAL_ERROR "unreachable: commit() accepted a restore that changed the baseline")
elseif(TXN_CASE STREQUAL "recover_rejects_unreadable_marker")
    reset_scratch()
    file(WRITE "${BASELINE_TXN_TARGET}" "${NARROW_CONTENT}")
    file(WRITE "${BASELINE_TXN_MARKER}" "MAYBE?\n")
    baseline_txn_recover()
    message(FATAL_ERROR "unreachable: recover() guessed at an unreadable marker")
elseif(TXN_CASE STREQUAL "recover_rejects_unremoved_absent_target")
    reset_scratch()
    file(MAKE_DIRECTORY "${BASELINE_TXN_TARGET}")
    file(WRITE "${BASELINE_TXN_TARGET}/narrow.txt" "${NARROW_CONTENT}")
    file(WRITE "${BASELINE_TXN_MARKER}" "ABSENT\n")
    baseline_txn_recover()
    message(FATAL_ERROR "unreachable: recover() accepted an unremoved ABSENT target")
elseif(TXN_CASE)
    message(FATAL_ERROR "unknown TXN_CASE '${TXN_CASE}'")
endif()

if(TXN_CASE)
    return()
endif()


# ---------------------------------------------------------------------------
# 1. Normal round trip over an existing baseline: byte-identical, no residue.
# ---------------------------------------------------------------------------
reset_scratch()
file(WRITE "${BASELINE_TXN_TARGET}" "${DEVELOPER_CONTENT}")
baseline_txn_begin()
file(WRITE "${BASELINE_TXN_TARGET}" "${NARROW_CONTENT}")
baseline_txn_commit()
expect_content("round trip restores the developer baseline" "${BASELINE_TXN_TARGET}" "${DEVELOPER_CONTENT}")
expect_no_open_transaction("round trip clears its state")
message(STATUS "PASS  existing baseline survives a --regenerate step byte for byte")

# ---------------------------------------------------------------------------
# 2. Normal round trip with no baseline: the narrow one the run created is
#    removed rather than left to shadow every row it does not name.
# ---------------------------------------------------------------------------
reset_scratch()
baseline_txn_begin()
file(WRITE "${BASELINE_TXN_TARGET}" "${NARROW_CONTENT}")
baseline_txn_commit()
expect_missing("round trip removes a baseline that did not exist" "${BASELINE_TXN_TARGET}")
expect_no_open_transaction("absent-baseline round trip clears its state")
message(STATUS "PASS  a baseline that did not exist is not left behind")

# ---------------------------------------------------------------------------
# 3. Interrupted transaction over an existing baseline: the next invocation
#    recovers it before anything destructive runs.
# ---------------------------------------------------------------------------
seed_interrupted_transaction(TRUE)
baseline_txn_recover()
expect_content("recovery restores the developer baseline" "${BASELINE_TXN_TARGET}" "${DEVELOPER_CONTENT}")
expect_no_open_transaction("recovery clears its state")
message(STATUS "PASS  an interrupted run's developer baseline is recovered")

# ---------------------------------------------------------------------------
# 4. Interrupted transaction where no baseline existed: the leftover narrow
#    file is removed. This is the case an in-memory flag cannot represent.
# ---------------------------------------------------------------------------
seed_interrupted_transaction(FALSE)
baseline_txn_recover()
expect_missing("recovery removes an interrupted run's narrow baseline" "${BASELINE_TXN_TARGET}")
expect_no_open_transaction("absent-baseline recovery clears its state")
message(STATUS "PASS  an interrupted run's narrow baseline is removed when none existed")

# ---------------------------------------------------------------------------
# 5. Recovery is idempotent, and a recovered transaction can be reopened.
# ---------------------------------------------------------------------------
seed_interrupted_transaction(TRUE)
baseline_txn_recover()
baseline_txn_recover()
expect_content("second recovery is a no-op" "${BASELINE_TXN_TARGET}" "${DEVELOPER_CONTENT}")
baseline_txn_begin()
file(WRITE "${BASELINE_TXN_TARGET}" "${NARROW_CONTENT}")
baseline_txn_commit()
expect_content("recovered baseline survives the next transaction" "${BASELINE_TXN_TARGET}" "${DEVELOPER_CONTENT}")
expect_no_open_transaction("post-recovery round trip clears its state")
message(STATUS "PASS  recovery is idempotent and reopens cleanly")

# ---------------------------------------------------------------------------
# 6. Refusals. Each of these would otherwise destroy the developer baseline
#    silently, so they must abort rather than proceed.
# ---------------------------------------------------------------------------
run_child_case("begin_over_unrecovered_backup" FALSE)
if(NOT CHILD_STDERR MATCHES "unrecovered baseline transaction")
    message(FATAL_ERROR "FAIL begin() aborted for the wrong reason:\n${CHILD_STDERR}")
endif()
message(STATUS "PASS  begin() refuses to overwrite an unrecovered snapshot")

run_child_case("commit_without_begin" FALSE)
if(NOT CHILD_STDERR MATCHES "no open transaction")
    message(FATAL_ERROR "FAIL commit() aborted for the wrong reason:\n${CHILD_STDERR}")
endif()
message(STATUS "PASS  commit() refuses to run without an open transaction")

run_child_case("commit_detects_corrupted_snapshot" FALSE)
if(NOT CHILD_STDERR MATCHES "not byte-identical")
    message(FATAL_ERROR "FAIL commit() did not machine-check before/after identity:\n${CHILD_STDERR}")
endif()
message(STATUS "PASS  commit() machine-checks that the baseline came back byte-identical")

run_child_case("recover_rejects_unreadable_marker" FALSE)
if(NOT CHILD_STDERR MATCHES "Unreadable baseline transaction marker")
    message(FATAL_ERROR "FAIL recover() did not reject an unreadable marker:\n${CHILD_STDERR}")
endif()
message(STATUS "PASS  recover() refuses to guess at an unreadable marker")

run_child_case("recover_rejects_unremoved_absent_target" FALSE)
if(NOT CHILD_STDERR MATCHES "Could not remove the baseline left by the interrupted run")
    message(FATAL_ERROR "FAIL recover() cleared an unremoved ABSENT target:\n${CHILD_STDERR}")
endif()
message(STATUS "PASS  recover() retains its marker when an ABSENT target cannot be removed")

file(REMOVE_RECURSE "${SCRATCH_DIR}")
message(STATUS "baseline transaction protocol: all cases passed")
